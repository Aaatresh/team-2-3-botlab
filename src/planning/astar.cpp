#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    // convert to cell x, y
    int x0 = pos_to_cell_x(start.x, distances);
    int x1 = pos_to_cell_x(goal.x, distances);
    int y0 = pos_to_cell_y(start.y, distances);
    int y1 = pos_to_cell_y(goal.y, distances);


    // perform Astar in the grid
    std::vector<Node> cellPath = search_for_path_grid(x0,y0,x1,y1, distances,params);

    // convert back to robot path
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    
    for (size_t i=0; i<= cellPath.size(); i++){
        pose_xyt_t pose;
        pose.x = cell_to_pos_x(cellPath[i].x, distances);
        pose.y = cell_to_pos_y(cellPath[i].y, distances);
        path.path.push_back(pose);
    }

    
    path.path_length = path.path.size();
    
    return path;
}

std::vector<Node> search_for_path_grid(int x0, int y0, int x1, int y1, 
                            const ObstacleDistanceGrid& distances,
                            const SearchParams& params){

    std::vector<Node> path;
    Node start_node;
    start_node.x = x0;
    start_node.y = y0;
    start_node.parent_x = x0; // make myself the parent
    start_node.parent_y = y0; // make myself the parent
    start_node.cost_g = 0.0; // cost so far
    start_node.cost_h = get_cost_h(start_node, x1, y1);
    start_node.cost_f = start_node.cost_g + start_node.cost_h;
    path.push_back(start_node);

    if ((x0 == x1) && (y0 == y1)){
        // already at goal!
        return path;
    }


    // initialise the queue
    std::priority_queue<Node> open_queue;
    // std::vector<Cell> closed_queue;
    bool closed_list[distances.widthInCells()][distances.heightInCells()];

    // actually run astar
    open_queue.push(start_node);

    while (open_queue.size() > 0){
        // int ind = get_next_node_ind(open_queue);
        Node node = open_queue.pop();
        for (auto k : get_children(open_queue)){

        }

    }




    return path;

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
float get_cost_h(Node n, int goal_x, int goal_y){
    return std::abs(goal_x - n.x) + std::abs(goal_y - n.y);
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
