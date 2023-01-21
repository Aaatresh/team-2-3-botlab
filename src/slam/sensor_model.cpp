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
    int rayStride = 20;

    
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, rayStride);

    for (auto&ray : movingScan){
        
        scanScore += scoreRay(ray, map);
    }
    
    return scanScore;
}

double SensorModel::scoreRay(adjusted_ray_t ray, OccupancyGrid map){
    // this is the method by gaskell

    // find the endpoint
    double end_x = ray.origin.x + ray.range * std::cos(ray.theta);
    double end_y = ray.origin.y + ray.range * std::sin(ray.theta);

    // convert to end_cell

    int x1 = map.pos_to_cell_x(end_x);
    int y1 = map.pos_to_cell_y(end_y);

    // get occupancy of (x1,y1)
    // return map.logOdds(x1,y1);
    bool occ = map.isOccupied(x1, y1);

    if (occ){
        return 1.0;
    }
    return 0.0;

}


