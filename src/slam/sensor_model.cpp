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
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, rayStride);

    for (size_t i=0; i<=movingScan.size(); i++){
    // for (auto&ray : movingScan){
        // compute ray score
        scanScore += scoreRay(movingScan[i], map);
    }
    
    return std::exp(scanScore);
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map){
 
    double s_too_near = -8;
    double s_correct = -4;
    double s_too_far = -12;


    // create expected endpoint from the ray
    // step using brensham from origin to expected endpoint
    // check if occupied for each cell along this ray
    // determine if early or late

    double threshold = 2*map.metersPerCell();

    Point<double> origin  = ray.origin;
    double ex = ray.origin.x + (ray.range+2*threshold) * std::cos(ray.theta);
    double ey = ray.origin.y + (ray.range+2*threshold) * std::sin(ray.theta);
    Point<double> endPoint(ex, ey);

    int x0 = map.pos_to_cell_x(origin.x);
    int x1 = map.pos_to_cell_x(endPoint.x);
    int y0 = map.pos_to_cell_y(origin.y);
    int y1 = map.pos_to_cell_y(endPoint.y);

    // start breshenham
    double dist_to_wall = get_dist_to_wall(x0, x1, y0, y1, map);

    if (ray.range <= dist_to_wall - threshold){
        return s_too_near;
    }
    if (ray.range >= dist_to_wall + threshold){
        return s_too_far;
    }

    return s_correct;
}


double SensorModel::get_dist_to_wall(int x0, int x1, int y0, int y1, OccupancyGrid& map){

    // breshenham
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;

    int err = dx - dy;

    int x = x0;
    int y = y0;

    while ((x != x1 ) || (y != y1)) {
        // check if cell is free

        if (!map.isCellInGrid(x, y)){
            break;
        }

        if (map.isOccupied(x, y)){
            break;
        }

        int e2 = 2 * err;
        if (e2 >= - dy){
            err -= dy;
            x += sx;
        }
        if (e2 <= dx){
            err += dx;
            y += sy;
        }
    
    }

    float DeltaX = map.metersPerCell() * (x-x0);
    float DeltaY = map.metersPerCell() * (y-y0);

    return std::sqrt(DeltaX*DeltaX + DeltaY*DeltaY);
    
}


