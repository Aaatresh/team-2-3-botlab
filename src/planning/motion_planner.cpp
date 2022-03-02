#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <lcmtypes/robot_path_t.hpp>
#include <cmath>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, 
                                     const pose_xyt_t& goal, 
                                     const SearchParams& searchParams) const
{
    // std::printf("in planPath");
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal))
    {
        robot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);

        std::cout << "INFO: path rejected due to invalid goal\n";        

        return failedPath;
    }
    
    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams);
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, const pose_xyt_t& goal) const
{
    std::printf("\n* planning path from %f, %f to %f, %f\n", start.x, start.y, goal.x, goal.y);
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const pose_xyt_t& goal) const
{
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);

    // std::cout << "got here!!\n";
    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    if(num_frontiers != 1 && distanceFromPrev < 2 * searchParams_.minDistanceToObstacle) return false;

    // std::cout << "first check passed!\n";
    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);

    // std::printf("DISTACES WIDTH: %d, HEIGHT: %d", distances_.widthInCells(), distances_.heightInCells());

    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        // float d = distances_(goalCell.x, goalCell.y);
        // float r = params_.robotRadius;

        // std::printf("Distances: %f, Radius: %f", d, r);

        // std::printf("goal dist: %f", distances_(goalCell.x, goalCell.y));
        std::printf(" YES IM IN THE GRID\n\n");
        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    

    std::cout << "****************** cell isnt in grid!! *********************\n";
    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const robot_path_t& path) const
{

    ///////////// TODO: Implement your test for a safe path here //////////////////

    return true;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 10.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 1.0;
}
