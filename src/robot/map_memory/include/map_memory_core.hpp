#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Initialize global map
    void initializeGlobalMap(double map_resolution, int map_width, int map_height);
    
    // Integrate costmap into global map using robot pose
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, 
                         const nav_msgs::msg::Odometry& odom);
    
    // Get the global map
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;

  private:
    rclcpp::Logger logger_;
    
    nav_msgs::msg::OccupancyGrid global_map_;
    bool map_initialized_;
    
    // Helper functions
    void worldToMap(double wx, double wy, int& mx, int& my) const;
    void mapToWorld(int mx, int my, double& wx, double& wy) const;
    int getCost(int mx, int my) const;
    void setCost(int mx, int my, int cost);
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;
};

}  

#endif  
