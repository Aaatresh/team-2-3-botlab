#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////

    double scanScore = 0.0;
    int rayStride = 10;
    MovingLaserScan movingScan(scan, sample.parent_pose, same.pose, rayStride);

    for (const auto&ray : movingScan){
        // compute ray score

        scanScore += scoreRay(ray, map);

    }    
    return scanScore;
}

double scoreRay(adjusted_ray& ray, OccupancyGrid& map){
 
    float s_too_near = -8;
    float s_correct = -4;
    float s_too_far = -12;

    double d = get_dist_from_map(ray, map);
    float thresh = map.metersPerCell_; # meters
    if (ray.range <= d - thresh){
        return s_too_near;
    }
    if (ray.range >= d + thresh){
        return s_too_far;
    }
    return s_correct;
}


double get_dist_from_map(adjusted_ray_t ray, OccupancyGrid& map){

    

}


