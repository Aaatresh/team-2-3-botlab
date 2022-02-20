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


    for(unsigned long i=0; i <= height_; i++){	

	for(unsigned long j=0; j <= width_; j++){
		
		if(isCellInGrid(j, i))
		{

			if(map.logOdds(j, i) >= 0)
			{
				distance(j, i) = 0;		
			}
			else
			{
				unsigned long add_factor = 1;
				while(1)
				{
					if((isCellInGrid(j + add_factor, i) && map.logOdds(j + add_factor, i) > 0) ||
					   (isCellInGrid(j, i + add_factor) && map.logOdds(j, i + add_factor) > 0) ||			
					   (isCellInGrid(j - add_factor, i) && map.logOdds(j - add_factor, i) > 0) ||			
					   (isCellInGrid(j, i - add_factor) && map.logOdds(j, i - add_factor) > 0))
					{
						distance(j, i) = add_factor;
						// printf("distance(%lu, %lu): %f\n", j, i, distance(j, i));
						break;
					}			
					else
						add_factor++;

					if(add_factor > std::max(height_, width_))
					{
						distance(j, i) = std::max(height_, width_);
						// printf("distance(%lu, %lu): %f\n", j, i, distance(j, i));
						break;
					}
	
				}	

			}

			// distance(i, j) = 0.4;
		
		}

	}

    }
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
