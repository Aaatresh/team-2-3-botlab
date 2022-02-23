#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


std::vector<Point<unsigned long>> ObstacleDistanceGrid::get_adj8(Point<unsigned long> boundary, unsigned long distance_counter){
    
    std::vector<Point<unsigned long>> adj8;

    j = boundary.x;
    i = boundary.y;

    if(distance_(j, i+1) == -1)
        adj8.push_back(Point<unsigned long>(j, i));

    if(distance_(j+1, i+1) == -1)
        adj8.push_back(Point<unsigned long>(j, i));
    
    if(distance_(j+1, i) == -1)
        adj8.push_back(Point<unsigned long>(j, i));

    if(distance_(j+1, i-1) == -1)
        adj8.push_back(Point<unsigned long>(j, i));
    
    if(distance_(j, i-1) == -1)
        adj8.push_back(Point<unsigned long>(j, i));

    if(distance_(j-1, i-1) == -1)
        adj8.push_back(Point<unsigned long>(j, i));

    if(distance_(j-1, i) == -1)
        adj8.push_back(Point<unsigned long>(j, i));

    if(distance_(j-1, i+1) == -1)
        adj8.push_back(Point<unsigned long>(j, i));


    return adj8;
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.


    // declare boundary and new_boundary
    std::vector<Point<unsigned long>> boundary, new_boundary, adj8;
    long distance_counter = 1;

    for(unsigned long i=0; i<= height_; i++){
	    
	    for(unsigned long j=0; j <= width_; j++){

		    if(map.logOdds(j, i) = 0){
			    distance_(j, i) = 0;
		    }
		    else if(map.logOdds(j, i) > 0){
			    distance_(j, i) = 0;
			    
			    boundary.push_back(Point(j, i));
		    }
		    else{
			    distance_(j, i) = -1;
		    }
	    }
    }

    while(1){
	    flag = 0;

	    while(boundary.empty() == false){
		    boundary_point = boundary.pop_back();

		    adj8 = get_adj8(boundary_point, distance_counter);

		    if(adj8.empty() == false){
		        flag = 1;

			for(auto& n : adj8){
			    j = n.x;
			    i = n.y;
			    
			    new_boundary.push_back(n);
			    distance_(j, i) = distance_counter;
			}
		    }
	    }

	    if(flag == 0){
		    break;
	    }

	    boundary = new_boundary;
	    new_boundary.clear();
	    distance_counter++;

	    if(distance_counter > std::max(height_, width_)){
		    distance_counter = std::max(height_, width_);
	    }

    }

    /*
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
						distance(j, i) = add_factor * 0.1;
						// printf("distance(%lu, %lu): %f\n", j, i, distance(j, i));
						break;
					}			
					else
						add_factor++;

					if(add_factor > std::max(height_, width_))
					{
						distance(j, i) = std::max(height_, width_) * 0.1;
						// printf("distance(%lu, %lu): %f\n", j, i, distance(j, i));
						break;
					}
	
				}	

			}

			// distance(i, j) = 0.4;
		
		}

	}

    }*/


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
    // cells_closed_.resize(width_ * height_);
}

// void ObstacleDistanceGrid::reset_closed_list(){
//     // reset all cells to be closed at the start
//     for (size_t i=0; i< cells_closed_.size(); i++){
//         cells_closed_[i] = false;
//     }
// }


// int ObstacleDistanceGrid::gridSize(){
//     return width_ * height_;
// }
