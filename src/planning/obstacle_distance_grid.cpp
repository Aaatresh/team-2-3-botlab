#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}

// int ObstacleDistanceGrid::pos_to_cell_x(float pos_x){
//     return int((pos_x - globalOrigin_.x ) * cellsPerMeter_);
// }
// int ObstacleDistanceGrid::pos_to_cell_y(float pos_y){
//     return int((pos_y - globalOrigin_.y ) * cellsPerMeter_);
// }
// float ObstacleDistanceGrid::cell_to_pos_x(int x){
//     return globalOrigin_.x + x * cellsPerMeter_;
// }
// float ObstacleDistanceGrid::cell_to_pos_y(int y){
//     return globalOrigin_.y + y * cellsPerMeter_;
// }
