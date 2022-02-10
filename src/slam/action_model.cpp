#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;

    numGen_ = std::mt19937 (rd());
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    
    if (!initialised_){
	 prevOdom_ = odometry;
         initialised_ = true;
    }

    double dx = odometry.x - prevOdom_.x;
    double dy = odometry.y - prevOdom_.y;
    double dth = angle_diff(odometry.theta, prevOdom_.theta);

    moved_ = (dx != 0.0) && (dy != 0.0) && (dth != 0.0);

    if (moved_){
        trans_= sqrt(dx*dx + dy*dy);
        rot1_ = angle_diff(std::atan2(dy, dx), prevOdom_.theta);
        rot2_ = angle_diff(dth, rot1_);
	
	rot1Std_ = sqrt(k1_ * std::abs(rot1_));
	rot2Std_ = sqrt(k1_ * std::abs(rot2_));
	transStd_ = sqrt(k2_ * std::abs(trans_));
    }

    prevOdom_ = odometry;
    utime_ = odometry.utime;

    return moved_;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.


   std::normal_distribution<float> d1(0.0, rot1Std_);
   std::normal_distribution<float> d2(0.0, transStd_);
   std::normal_distribution<float> d3(0.0, rot2Std_);

    float e1 = d1(numGen_);
    float e2 = d2(numGen_);
    float e3 = d3(numGen_);

   particle_t newSample = sample;
   
   newSample.pose.x += (trans_+e2) * std::cos(sample.pose.theta + rot1_ + e1);
   newSample.pose.y += (trans_+e2) * std::sin(sample.pose.theta + rot1_ + e1);
   newSample.pose.theta += rot1_ + rot2_ + e1 + e3;
   newSample.pose.utime = utime_;
   newSample.pose.theta = wrap_to_pi(newSample.pose.theta);


   newSample.parent_pose = sample.pose;

    return newSample;

}
