#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
    // max_cell_distance = width_ * height_;
    // max_cell_distance = -1;
}


std::vector<Point<unsigned int>> ObstacleDistanceGrid::get_adj8(Point<unsigned int> boundary, unsigned int distance_counter){

    
    std::vector<Point<unsigned int>> adj8;

    unsigned int j = boundary.x;
    unsigned int i = boundary.y;

    if(isCellInGrid(j, i+1) && distance(j, i+1) == max_cell_distance)
        adj8.push_back(Point<unsigned int>(j, i+1));

    if(isCellInGrid(j+1, i+1) && distance(j+1, i+1) == max_cell_distance)
        adj8.push_back(Point<unsigned int>(j+1, i+1));
    
    if(isCellInGrid(j+1, i) && distance(j+1, i) == max_cell_distance)
        adj8.push_back(Point<unsigned int>(j+1, i));

    if(isCellInGrid(j+1, i-1) && distance(j+1, i-1) == max_cell_distance)
        adj8.push_back(Point<unsigned int>(j+1, i-1));
    
    if(isCellInGrid(j, i-1) && distance(j, i-1) == max_cell_distance)
        adj8.push_back(Point<unsigned int>(j, i-1));

    if(isCellInGrid(j-1, i-1) && distance(j-1, i-1) == max_cell_distance)
        adj8.push_back(Point<unsigned int>(j-1, i-1));

    if(isCellInGrid(j-1, i) && distance(j-1, i) == max_cell_distance)
        adj8.push_back(Point<unsigned int>(j-1, i));

    if(isCellInGrid(j-1, i+1) && distance(j-1, i+1) == max_cell_distance)
        adj8.push_back(Point<unsigned int>(j-1, i+1));


    return adj8;
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);

    max_cell_distance = width_ * height_;

    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.

    // declare boundary and new_boundary
    std::vector<Point<unsigned int>> boundary, new_boundary, adj8;
    
    unsigned int distance_counter = 1;

    for(unsigned int i=0; i<= height_; i++){
	    
	    for(unsigned int j=0; j <= width_; j++){
		    
                    if(isCellInGrid(j, i)){
			
			
			if(map.logOdds(j, i) == 0){
				// distance(j, i) = 0;
			}
		    	else if(map.logOdds(j, i) > 0){
				distance(j, i) = 0;
			    
			    	boundary.push_back(Point<unsigned int>(j, i));
		    	}
		    	else{
				distance(j, i) = max_cell_distance;
		    	}

		}
	    }
    }


    while(1){
	    int flag = 0;

	    char input = 0;

	    for(int k=0; k < boundary.size(); k++){

		    Point<unsigned int> boundary_point = boundary[k];

		    adj8 = get_adj8(boundary_point, distance_counter);

		    if(adj8.empty() == false){
		        flag = 1;

			for(auto& n : adj8){
			    unsigned int j = n.x;
			    unsigned int i = n.y;
			    
			    new_boundary.push_back(n);
			    distance(j, i) = distance_counter * metersPerCell_;
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
