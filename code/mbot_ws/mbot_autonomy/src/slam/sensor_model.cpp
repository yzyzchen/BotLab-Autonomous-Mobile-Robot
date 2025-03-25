#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    bfs_offsets_.clear();

    for(int i = -search_range; i <= search_range; i++){
        for(int j = -search_range; j <= search_range; j++){
            if(i == 0 && j == 0){
                continue;
            }
            bfs_offsets_.push_back(Point<int>(i,j));
        }
    }

    Point<int> center(0, 0);
    sort(bfs_offsets_.begin(), bfs_offsets_.end(), [&](const Point<int>& a, const Point<int>& b) {
        return distance_between_points(a, center) <= distance_between_points(b, center);
    });
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the likelihood of the given particle using the provided laser scan and map. 
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    double scanScore = 0.0;

    for(auto& ray : movingScan){
        // if(ray.range > max_ray_range_){
        //     continue;
        // }
        scanScore += scoreRay(ray, map);
        // auto rayend = global_position_to_grid_cell(getRayEndPointOnMap(ray, map), map);
        // scanScore += map.logOdds(rayend.x, rayend.y);
    }

    return scanScore;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Compute a score for a given ray based on its end point and the map. 
    // Consider the offset from the nearest occupied cell.  
    auto rayEnd = global_position_to_grid_cell(getRayEndPointOnMap(ray, map), map);
    auto nearestOccupied = gridBFS(rayEnd, map);
    auto offset = rayEnd - nearestOccupied;
    auto dist = sqrt(offset.x*offset.x + offset.y*offset.y);
    auto quality = NormalPdf(dist) * offset_quality_weight;

    return quality;
}

double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 
    if(map.logOdds(end_point.x, end_point.y) >= occupancy_threshold_){
        return end_point;
    }

    for(auto& offset : bfs_offsets_){
        auto next = end_point + offset;
        if(map.logOdds(next.x, next.y) >= occupancy_threshold_){
            return next;
        }
    }

    return Point<int>(-1,-1);  
}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Calculate the end point of a given ray on the map 
    Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
                           ray.origin.y + ray.range * std::sin(ray.theta));
    return endpoint;
}
