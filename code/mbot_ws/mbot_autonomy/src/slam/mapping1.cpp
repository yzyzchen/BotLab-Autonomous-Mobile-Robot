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
    if (!initialized_) {
        initialized_ = true;
        previousPose_ = pose;
    }

    MovingLaserScan movingScan(scan, previousPose_, pose);

    for (auto& ray : movingScan) {
        scoreEndpoint(ray, map);
    }

    for (auto& ray : movingScan) {
        scoreRay(ray, map);
    }

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    // Make sure ray is in range
    if (ray.range >= kMaxLaserDistance_ - 0.2f || ray.range <= 0.05f) {
        return;
    }

    Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
    Point<int> rayCell;

    rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
    rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

    if (map.isCellInGrid(rayCell.x, rayCell.y)) {
        increaseCellOdds(rayCell.x, rayCell.y, map);
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    // Make sure ray is in range
    if (ray.range >= kMaxLaserDistance_ - 0.1f) {
        return;
    }

    std::vector<Point<int>> emptyCells = bresenham(ray, map);

    for (int i = 0; i < emptyCells.size() - 1; i++) {
        auto cell = emptyCells[i];
        if (map.isCellInGrid(cell.x, cell.y)) {
            decreaseCellOdds(cell.x, cell.y, map);
        }
    }
}

int sign(float x) {
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Convert ray to start and end

    Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
    Point<float> rayEnd;
    rayEnd.x = (ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x;
    rayEnd.y = (ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y;

    int x0 = static_cast<int>(rayStart.x);
    int y0 = static_cast<int>(rayStart.y);

    int x1 = static_cast<int>(rayEnd.x);
    int y1 = static_cast<int>(rayEnd.y);

    // Initializing variables
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = sign(x1 - x0);
    int sy = sign(y1 - y0);

    int err = dx - dy;
    int e2;

    int x = x0;
    int y = y0;

    std::vector<Point<int>> points;
    points.push_back(Point<int>(x, y));

    // Main algorithm
    while ((x != x1) || (y != y1)) {
        e2 = 2 * err;
        if (e2 >= -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
        points.push_back(Point<int>(x, y));
    }

    return points;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray.
    return {};
    
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map) {
    if (!initialized_) {
        return;
    }
    int8_t newOdds = map(x,y) + kHitOdds_;

    // Overflow correction
    if (newOdds < map(x,y)) {
        newOdds = 127;
    }

    map(x,y) = newOdds;
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map) {
    if (!initialized_) {
        return;
    }
    
    int8_t newOdds = map(x,y) - kMissOdds_;

    // Underflow correction
    if (newOdds > map(x,y)) {
        newOdds = -127;
    }

    map(x,y) = newOdds;
}