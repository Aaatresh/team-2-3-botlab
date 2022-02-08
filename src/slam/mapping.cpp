#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}



void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////

    // printf("RECEIVED NEW SCAN!\n");
    printf("New Pose: %f, %f, %f", pose.x, pose.y, pose.theta);

    // interpolate
    MovingLaserScan adj_scan = MovingLaserScan(scan,pose, pose, 1);
    
    // int x0 = map.pos_to_cell_x(pose.x);
    // int y0 = map.pos_to_cell_y(pose.y);

    // printf("%f, %f : %d, %d", pose.x, pose.y, x0, y0);

    for (size_t n=0; n<adj_scan.size(); n++){

        float pos_x0 = adj_scan[n].origin.x;
        float pos_y0 = adj_scan[n].origin.y;


        float pos_x1 = pos_x0 + adj_scan[n].range * cos(adj_scan[n].theta);
        float pos_y1 = pos_y0 + adj_scan[n].range * sin(adj_scan[n].theta);


        int x0 = map.pos_to_cell_x(pos_x0);
        int y0 = map.pos_to_cell_y(pos_y0);
        int x1 = map.pos_to_cell_x(pos_x1);
        int y1 = map.pos_to_cell_y(pos_y1);

        // printf("%f, %f : %d, %d\n", pos_x1, pos_y1, x1, y1);
        
        Mapping::insertRayCells(x0,y0,x1,y1, true, map);
    }

    // printf("DONE INSERTING!");
    std::cout << std::endl;

    
    
    map.saveToFile("current.map");
    // printf("SAVED!");

    return;

}


// inserts a ray into the occupancy grid, starting at cell (x0, y0) and ending at (x1, y1)
// if end_obstacle=True, assumes the end point is a hit, else it assumes it is free
void Mapping::insertRayCells(int x0, int y0, int x1, int y1, bool end_is_obstacle, OccupancyGrid& map){

    // use breshenham's algorithm to mark the free cells
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;

    int err = dx - dy;

    int x = x0;
    int y = y0;

    while ((x != x1 ) || (y != y1)) {
        // insert cells as free
        
        Mapping::updateOdds(map, x, y, false);
        int e2 = 2 * err;
        if (e2 >= - dy){
            err -= dy;
            x += sx;
        }
        if (e2 <= dx){
            err += dx;
            y += sy;
        }
    
    }

    // insert the last point as a hit
    Mapping::updateOdds(map, x, y, end_is_obstacle);

    return;

}


CellOdds safeAdd(CellOdds o1, CellOdds o2){
        if (o1 < std::numeric_limits<CellOdds>::max() - o2 ){
            return o1 + o2;
        }
        else{
            return std::numeric_limits<CellOdds>::max();
        }
}

CellOdds safeMinus(CellOdds o1, CellOdds o2){
    // returns o1 - o2
        if (o1 > std::numeric_limits<CellOdds>::min() + o2){
            return o1 - o2;
        }
        else{
            return std::numeric_limits<CellOdds>::min();
        }
}

void Mapping::updateOdds(OccupancyGrid& map, int x, int y, bool occupied){

    if (occupied == true){
        // set log-odds to hit
        CellOdds logOdds = safeAdd(map.logOdds(x, y), Mapping::kHitOdds_);
        map.setLogOdds(x, y,logOdds);
    }
    if (occupied == false){
        // set log-odds to false
        CellOdds logOdds = safeMinus(map.logOdds(x, y), Mapping::kMissOdds_);
        map.setLogOdds(x, y,logOdds);
    }
}


