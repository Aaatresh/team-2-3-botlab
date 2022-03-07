#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

// #define DIAGONAL // comment this line to disable moving in diagonals

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include<queue>
#include <tuple>
#include <algorithm>
#include <common/grid_utils.hpp>

// struct CellPath{
//     st
//     // std::vector<int> xs;
//     // std::vector<int> ys;
// };

// create custom types
typedef std::pair<int, int> Pair;
// struct Pair{
//     int x;
//     int y;
//     Pair(int xx, int yy)
//         : x(xx)
//         , y(yy)
//     {}
// };
typedef std::tuple<double, int, int> Tuple;

struct Cell {
    Pair parent;
    float f, g, h;
    // default constructor is outside cells space
    Cell()
        : parent(-1, -1)
        , f(-1)
        , g(-1)
        , h(-1)
    {}
};

inline bool operator < (const Cell& lhs, const Cell& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.f < rhs.f;
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


std::vector<Pair> search_for_path_grid(Pair start, Pair Goal,
                            const ObstacleDistanceGrid& distances,
                            const SearchParams& params);


float get_cost_h(Pair n, Pair goal);

int get_index(int x, int y, int width);

bool isDestination(Pair n, Pair goal);

std::vector<Pair> neighbors(int x, int y, const ObstacleDistanceGrid& distances);

std::vector<Pair> tracePath(std::vector<Cell> cellDetails, Pair goal, int width);

int get_dir(Pair p0, Pair p1);
std::vector<Pair> simplifyPath(std::vector<Pair> originalPath);


#endif // PLANNING_ASTAR_HPP