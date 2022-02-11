#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <common/angle_functions.hpp>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////

    double sampleWeight = 1.0 / kNumParticles_;

    posteriorPose_ = pose;

    for(auto& p:posterior_){
	p.pose.x = posteriorPose_.x;
	p.pose.y = posteriorPose_.y;
	p.pose.theta = wrap_to_pi(posteriorPose_.theta);
	p.pose.utime = pose.utime;
	p.parent_pose = p.pose;
	p.weight = sampleWeight;
	}

}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior = posterior_;

    double sampleWeight = 1.0/kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<float> dist_pos(0.0, 0.05); // 5 cm std
    std::normal_distribution<float> dist_angle(0.0, 2.0*M_PI/180.0); // 5degrees std

    for (auto& p : prior){
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist(generator));
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = posteriorPose_;
        p.weight = sampleWeight;
    }


    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    for(auto& p:prior){
	    proposal.push_back(actionModel_.applyAction(p));
    }

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    float total_sum = 0.0f;

    for (auto& p:proposal){

        particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        total_sum += weighted.weight;

        posterior.push_back(weighted);
        
    }

    for (auto& p: posterior){
        p.weight /= total_sum;
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;

    // SIMPLE AVERAGE

    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_cos = 0.0;
    double mean_sin = 0.0;

    for (auto&p : posterior){
        mean_x += p.pose.x * p.weight;
        mean_y += p.pose.y * p.weight;
        mean_cos += std::cos(p.pose.theta) * p.weight;
        mean_sin += std::sin(p.pose.theta) * p.weight;
    }

    pose.x = mean_x;
    pose.y = mean_y;
    pose.theta = std::atan2(mean_sin, mean_cos);

    return pose;
}
