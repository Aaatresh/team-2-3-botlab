#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>




robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    auto startPoint = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    auto goalPoint = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);

    // perform Astar in the grid
    Pair startCell(startPoint.x, startPoint.y);
    Pair goalCell(goalPoint.x, goalPoint.y);
    std::vector<Pair> cellPath = simplifyPath(search_for_path_grid(startCell, goalCell, distances,params));

    // convert back to robot path
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start); 
    // std::printf("Adding Pose: %f, %f\n", start.x, start.y);   
    
    if (cellPath.size() != 0){
        for (size_t i = 0; i< cellPath.size(); i++){
            pose_xyt_t pose;
            float x = 1.0*cellPath[cellPath.size() - i - 1].first;
            float y = 1.0*cellPath[cellPath.size() - i - 1].second;
            auto point = grid_position_to_global_position(Point<double>(x, y), distances);
            pose.x = point.x;
            pose.y = point.y;
            path.path.push_back(pose);
            // std::printf("Adding Pose: %f, %f\n", pose.x, pose.y);
        }
    }

    path.path_length = path.path.size();
    
    return path;
}

int get_index(int x, int y, int width){
    return y*width + x; 
}



std::vector<Pair> search_for_path_grid(Pair start, Pair goal,
                            const ObstacleDistanceGrid& distances,
                            const SearchParams& params)
{

    std::printf("GRID: REQUESTED start %d, %d, goal %d, %d\n", start.first, start.second, goal.first, goal.second);
    std::printf("MIN DIST: %f", params.minDistanceToObstacle);
    std::vector<Pair> path;

    int gridSize = distances.gridSize();
    int const mapWidth = distances.widthInCells();
    // reset all cells to closed
    std::vector<bool> closed_list(gridSize, false);

    // create cell-details array
    std::vector<Cell> cellDetails (gridSize);
    // cellDetails.resize(gridSize);

    // Initialise the parameters of the starting node
    int i = start.first;
    int j = start.second;
    int ind = get_index(i, j, mapWidth);
    cellDetails[ind].f = 0.0;
    cellDetails[ind].g = 0.0;
    cellDetails[ind].h = 0.0;
    cellDetails[ind].parent = {i, j};

    // create a priority list
    // each element of the queue is a tuple (f, x, y)
    std::priority_queue<Tuple, std::vector<Tuple>, std::greater<Tuple> > openList;

    // place first element on the queue    
    openList.emplace(0.0, start.first, start.second); // create the first node, and place it in the queue

    while (! openList.empty())
    {
        const Tuple& p = openList.top(); // get the next element from the queue
        i = std::get<1>(p);
        j = std::get<2>(p);
        ind = get_index(i, j, mapWidth);

        openList.pop();
        closed_list[ind] = true;

        // generate all neighbors
        for (Pair neighbour : neighbors(i, j, distances))
        {
            // If the destination cell is the same
            // as the current successor
            int n_ind = get_index(neighbour.first, neighbour.second, mapWidth);

            if (isDestination(neighbour, goal)) { // Set the Parent of the destination cell
                cellDetails[n_ind].parent = { i, j };
                // printf("The destination cell is found\n");
                return tracePath(cellDetails, neighbour, mapWidth);
            }
            // If the successor is already on the closed list or if it is blocked, then ignore it.  Else do the following
            
            if (!closed_list[n_ind] && (distances(neighbour.first, neighbour.second) > 1.414*params.minDistanceToObstacle)) {
                // std::printf("Cell: %d, %d, D: %f\n", neighbour.first, neighbour.second, distances(neighbour.first, neighbour.second));
                double gNew, hNew, fNew;
                // #ifdef DIAGONAL
                    // float dx = neighbour.first - i;
                    // float dy = neighbour.second - j;
                    // gNew = cellDetails[ind].g + std::sqrt(dx*dx + dy*dy);
                // #endif
                // #ifndef DIAGONAL
                    gNew = cellDetails[ind].g + 1.0;
                // #endif
                hNew = get_cost_h(neighbour, goal);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add
                // it to the open list. Make the
                // current square the parent of this
                // square. Record the f, g, and h
                // costs of the square cell
                //             OR
                // If it is on the open list
                // already, check to see if this
                // path to that square is better,
                // using 'f' cost as the measure.
                if (cellDetails[n_ind].f== -1 || cellDetails[n_ind].f > fNew) {
                    
                    openList.emplace(fNew, neighbour.first, neighbour.second);

                    // Update the details of this cell
                    cellDetails[n_ind].g = gNew;
                    cellDetails[n_ind].h = hNew;
                    cellDetails[n_ind].f = fNew;
                    cellDetails[n_ind].parent= { i, j };
                }
            }
        }
    }

    std::printf("NO VALID PATH FOUND!!");

    return path;

}

bool isDestination(Pair n, Pair goal){
    return (n.first == goal.first) && (n.second == goal.second);
}

std::vector<Pair> tracePath(std::vector<Cell> cellDetails, Pair goal, int width){
    std::vector<Pair> path;

    Pair c = goal;
    path.push_back(c);

    bool done = false;

    while (!done){
        
        int c_ind = get_index(c.first, c.second, width);
        Pair p = cellDetails[c_ind].parent;

        if ((p.first == c.first) && (p.second == c.second)){
            // done
            done = true;
            return path;
        }
        path.push_back(p);
        c = p;
    }
    
    return path;

}

// non-diagonal version
std::vector<Pair> neighbors(int x, int y, const ObstacleDistanceGrid& distances){
    // return valid neighbors
    std::vector<Pair> pairs;
    std::vector<Pair> dirs;
    // const bool diagonal = true;

    // ONLY LEFT RIGHT UP DOWN
    dirs.push_back(Pair(x+1, y));
    dirs.push_back(Pair(x, y+1));
    dirs.push_back(Pair(x-1, y));
    dirs.push_back(Pair(x, y-1));

    // ALL FOUR DIRECTIONS
    // dirs.push_back(Pair(x+1, y));
    // dirs.push_back(Pair(x+1, y+1));
    // dirs.push_back(Pair(x, y+1));
    // dirs.push_back(Pair(x-1, y+1));
    // dirs.push_back(Pair(x-1, y));
    // dirs.push_back(Pair(x-1, y-1));
    // dirs.push_back(Pair(x, y-1));
    // dirs.push_back(Pair(x+1, y-1));

    for (auto d : dirs){
        if (distances.isCellInGrid(d.first, d.second)){
            Pair p(d.first, d.second);
            pairs.push_back(p);
        }
    }

    return pairs;
}


// return the estimated cost to go
float get_cost_h(Pair n, Pair goal){
    // return std::sqrt(std::pow(goal.first - n.first, 2) + std::pow(goal.second - n.second, 2));
    return std::abs(goal.first - n.first) + std::abs(goal.second - n.second);
}


std::vector<Pair> simplifyPath(std::vector<Pair> originalPath){

    if (originalPath.size() <= 1) return originalPath;

    std::vector<Pair> newPath;

    // newPath.push_back(originalPath[0]);
    int dir = -1; //get_dir(originalPath[0], originalPath[1]);
    
    for (size_t n = 0; n < originalPath.size()-1; n++){
        int newDir = get_dir(originalPath[n], originalPath[n+1]);
        // std::printf("old dir: %d NEW DIR: %d\n", dir, newDir);
        if (newDir != dir){
            // std::printf("PUSHING\n");

            newPath.push_back(originalPath[n]);
            dir = newDir;
        }

    }
    newPath.push_back(originalPath.back());

    std::printf("ORIGINAL_LENGTH: %d, NEW_LENGTH: %d \n \n ", originalPath.size(), newPath.size());

    return newPath;

}

int get_dir(Pair p0, Pair p1){
    if (p0.first == p1.first + 1 && p0.second == p1.second) return 1;
    if (p0.first == p1.first - 1 && p0.second == p1.second) return 2;
    if (p0.first == p1.first && p0.second == p1.second + 1) return 3;
    if (p0.first == p1.first && p0.second == p1.second - 1) return 4;
    return 5;
}