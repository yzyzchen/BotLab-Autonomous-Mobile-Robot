#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose); 

    /// TODO: Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.
    for (auto& ray : movingScan){
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits  
    if(ray.range < kMaxLaserDistance_){
        Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;
        
        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            if((std::numeric_limits<CellOdds>::max() - map.logOdds(rayCell.x, rayCell.y)) > kHitOdds_){
                map.setLogOdds(rayCell.x, rayCell.y, (map.logOdds(rayCell.x, rayCell.y) + kHitOdds_));
            }
            else{
                map.setLogOdds(rayCell.x, rayCell.y, std::numeric_limits<CellOdds>::max());
            }
        }
    }    
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through  
    if(ray.range < kMaxLaserDistance_){
        std::vector<Point<int>> midCells = bresenham(ray, map);
        for(auto& gridCell : midCells){
            if(map.isCellInGrid(gridCell.x, gridCell.y)){
                if((map.logOdds(gridCell.x, gridCell.y) - std::numeric_limits<CellOdds>::min()) > kMissOdds_){
                    map.setLogOdds(gridCell.x, gridCell.y, (map.logOdds(gridCell.x, gridCell.y) - kMissOdds_));
                }
                else{
                    map.setLogOdds(gridCell.x, gridCell.y, std::numeric_limits<CellOdds>::min());
                }
            }
        }
    }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    std::vector<Point<int>> cells;

    Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
    Point<int> rayEnd;
    
    rayEnd.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    rayEnd.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);
    
    Point<int> diff, sign, current;

    diff.x = abs(rayEnd.x - rayStart.x);
    diff.y = abs(rayEnd.y - rayStart.y);

    sign.x = ((rayEnd.x - rayStart.x) > 0) ? 1 : -1;
    sign.y = ((rayEnd.y - rayStart.y) > 0) ? 1 : -1;

    current.x = rayStart.x;
    current.y = rayStart.y;
    int error = diff.x - diff.y;

    cells.push_back(current);
    while((current.x != rayEnd.x) || (current.y != rayEnd.y)){
        int double_error = 2*error;
        if(double_error >= -diff.y){
            error -= diff.y;
            current.x += sign.x;
        }
        if(double_error <= diff.x){
            error += diff.x;
            current.y += sign.y;
        }
        cells.push_back(current);
    }
    cells.pop_back();
    return cells;
    
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray. 
    return {};
    
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map) {
    /// TODO: Increase the odds of the cell at (x,y)
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map) {
    /// TODO: Decrease the odds of the cell at (x,y)
}
