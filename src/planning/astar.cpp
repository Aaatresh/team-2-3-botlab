#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>




robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    // std::printf("RUNNING ASTAR!\n");
    // std::printf("Starting Astar, using minDist: %f\n", params.minDistanceToObstacle);

    auto startPoint = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    auto goalPoint = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);


    // perform Astar in the grid
    Pair startCell(startPoint.x, startPoint.y);
    Pair goalCell(goalPoint.x, goalPoint.y);
    std::vector<Pair> cellPath = search_for_path_grid(startCell, goalCell, distances,params);

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

    // // printf("length: %d\n", path.path_length);
    // // printf("START POSE: REQUESTED: %f, %f, FOUND: %f, %f\n", start.x, start.y, path.path[0].x, path.path[0].y);
    // // printf("END POSE: REQUESTED: %f, %f, FOUND: %f, %f\n", goal.x, goal.y, path.path[path.path_length-1].x, path.path[path.path_length-1].y);
    
    
    
    return path;
}

int get_index(int x, int y, int width){
    return y*width + x; 
}

std::vector<Pair> search_for_path_grid(Pair start, Pair goal,
                            const ObstacleDistanceGrid& distances,
                            const SearchParams& params)
{
    std::vector<Pair> path;

    // TODO: check validity of start and goal

    int gridSize = distances.gridSize();
    int const mapWidth = distances.widthInCells();
    // reset all cells to closed
    std::vector<bool> closed_list;
    closed_list.resize(gridSize);
    for (size_t i = 0; i<= closed_list.size(); i++){
        closed_list[i] = false;
    }

    // create cell-details array
    std::vector<Cell> cellDetails;
    cellDetails.resize(gridSize);

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

    // std::printf("NO VALID PATH FOUND!!");

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

        // printf("CELL: %d, %d\n", c.first, c.second);
        
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
    // std::reverse(path.begin(),path.end());
    return path;


}

// // diagonal version
// std::vector<Pair> neighbors(int x, int y, const ObstacleDistanceGrid& distances){
//     // return valid neighbors
//     std::vector<Pair> pairs;
//     for (int i=-1; i<= 1; i++){
//         for (int j=-1; j<= 1; j++){
//             if ((i!=0) && (j!=0)){
//                 if (distances.isCellInGrid(x+i, y+j)){
//                     Pair p(x+i, y+j);
//                     pairs.push_back(p);
//                 }
//             }
//         }
//     }
//     return pairs;
// }



// non-diagonal version
std::vector<Pair> neighbors(int x, int y, const ObstacleDistanceGrid& distances){
    // return valid neighbors
    std::vector<Pair> pairs;
    // std::vector<Pair> dirs;
    // const bool diagonal = true;

    // dirs.push_back(Pair(x+1, y));
    // dirs.push_back(Pair(x-1, y));
    // dirs.push_back(Pair(x, y+1));
    // dirs.push_back(Pair(x, y-1));
    
    // #ifdef DIAGONAL
    //     dirs.push_back(Pair(x+1, y+1));
    //     dirs.push_back(Pair(x-1, y+1));
    //     dirs.push_back(Pair(x+1, y-1));
    //     dirs.push_back(Pair(x-1, y-1));
    // #endif

    // for (auto d : dirs){
    //     if (distances.isCellInGrid(d.first, d.second)){
    //         Pair p(d.first, d.second);
    //         pairs.push_back(p);
    //     }
    // }

    // +x 
    if (distances.isCellInGrid(x + 1, y)){
        Pair p(x+1, y);
        pairs.push_back(p);
    }

    // -x 
    if (distances.isCellInGrid(x - 1, y)){
        Pair p(x-1, y);
        pairs.push_back(p);
    }
    
    // +y 
    if (distances.isCellInGrid(x, y+ 1)){
        Pair p(x, y+1);
        pairs.push_back(p);
    }

    // -y
    if (distances.isCellInGrid(x, y-1)){
        Pair p(x, y-1);
        pairs.push_back(p);
    }

    // // +x+y
    // if (distances.isCellInGrid(x+1, y+1)){
    //     Pair p(x+1, y+1);
    //     pairs.push_back(p);
    // }

    // // +x-y
    // if (distances.isCellInGrid(x+1, y-1)){
    //     Pair p(x+1, y-1);
    //     pairs.push_back(p);
    // }

    // // -x+y
    // if (distances.isCellInGrid(x-1, y+1)){
    //     Pair p(x-1, y+1);
    //     pairs.push_back(p);
    // }

    // // -x+y
    // if (distances.isCellInGrid(x-1, y-1)){
    //     Pair p(x-1, y-1);
    //     pairs.push_back(p);
    // }

    

    return pairs;
}


// int get_next_node_ind(std::vector<Node> open_queue){
//     float lowest_cost = 1e6;
//     int ind = 0;
//     for (size_t i = 0; i<= open_queue.size(); i++ ){
//         if (open_queue[i].cost_f <= lowest_cost){
//             lowest_cost = open_queue[i].cost_f
//             ind = i
//         }
//     }
//     open_queue.
//     return open_queue.pop();
// }


// return the estimated cost to go
float get_cost_h(Pair n, Pair goal){
    return std::sqrt(std::pow(goal.first - n.first, 2) + std::pow(goal.second - n.second, 2));
    // return std::abs(goal.first - n.first) + std::abs(goal.second - n.second);
}






int pos_to_cell_x(float pos_x, const ObstacleDistanceGrid& distances){
    return int((pos_x - distances.originInGlobalFrame().x ) * distances.cellsPerMeter());
}

int pos_to_cell_y(float pos_y, const ObstacleDistanceGrid& distances){
    return int((pos_y - distances.originInGlobalFrame().y ) * distances.cellsPerMeter());
}

float cell_to_pos_x(int x, const ObstacleDistanceGrid& distances){
    return distances.originInGlobalFrame().x + x * distances.metersPerCell();
}

float cell_to_pos_y(int y, const ObstacleDistanceGrid& distances){
    return distances.originInGlobalFrame().y + y * distances.metersPerCell();
}
