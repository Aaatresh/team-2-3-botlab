#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include<queue>

// struct CellPath{
//     st
//     // std::vector<int> xs;
//     // std::vector<int> ys;
// };

struct Node{
    int x;
    int y;
    int parent_x;
    int parent_y;
    float cost_f;
    float cost_g;
    float cost_h;
};

struct Cell{
    int x; 
    int y;
};


inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.cost_f < rhs.cost_f;
}

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};


/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);


std::vector<Node> search_for_path_grid(int x0, int y0, int x1, int y1, 
                            const ObstacleDistanceGrid& distances,
                            const SearchParams& params);




int pos_to_cell_x(float pos_x, const ObstacleDistanceGrid& distances);

int pos_to_cell_y(float pos_y, const ObstacleDistanceGrid& distances);

float cell_to_pos_x(int x, const ObstacleDistanceGrid& distances);

float cell_to_pos_y(int y, const ObstacleDistanceGrid& distances);

float get_cost_h(Node n, int goal_x, int goal_y);

Node get_next_node(std::vector<Node> open_queue);

#endif // PLANNING_ASTAR_HPP