#include "map_memory_core.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger), map_initialized_(false) {}

void MapMemoryCore::initializeGlobalMap(double map_resolution, int map_width, int map_height) {
    global_map_.info.resolution = map_resolution;
    global_map_.info.width = map_width;
    global_map_.info.height = map_height;
    global_map_.info.origin.position.x = -(map_width * map_resolution) / 2.0;
    global_map_.info.origin.position.y = -(map_height * map_resolution) / 2.0;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.resize(map_width * map_height, -1);  // -1 = unknown
    global_map_.header.frame_id = "map";
    map_initialized_ = true;
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, 
                                     const nav_msgs::msg::Odometry& odom) {
    if (!map_initialized_) {
        // Initialize with larger size than costmap (as per documentation)
        double costmap_resolution = costmap.info.resolution;
        int costmap_width = costmap.info.width;
        int costmap_height = costmap.info.height;
        
        // Make global map larger and with same or finer resolution
        double map_resolution = costmap_resolution;  // Same resolution
        int map_width = costmap_width * 3;  // 3x larger
        int map_height = costmap_height * 3;
        
        initializeGlobalMap(map_resolution, map_width, map_height);
    }
    
    // Get robot position
    double robot_x = odom.pose.pose.position.x;
    double robot_y = odom.pose.pose.position.y;
    double robot_yaw = extractYaw(odom.pose.pose.orientation);
    
    // Get costmap origin in costmap frame
    double costmap_origin_x = costmap.info.origin.position.x;
    double costmap_origin_y = costmap.info.origin.position.y;
    
    // Transform each cell from costmap to global map
    for (int cy = 0; cy < static_cast<int>(costmap.info.height); ++cy) {
        for (int cx = 0; cx < static_cast<int>(costmap.info.width); ++cx) {
            // Get costmap cell value
            int costmap_idx = cy * costmap.info.width + cx;
            int8_t costmap_value = costmap.data[costmap_idx];
            
            // Skip unknown cells
            if (costmap_value == -1) {
                continue;
            }
            
            // Convert costmap cell to world coordinates (in costmap frame)
            double costmap_wx = costmap_origin_x + (cx + 0.5) * costmap.info.resolution;
            double costmap_wy = costmap_origin_y + (cy + 0.5) * costmap.info.resolution;
            
            // Transform to global map frame (rotate and translate)
            double cos_yaw = std::cos(robot_yaw);
            double sin_yaw = std::sin(robot_yaw);
            double global_wx = robot_x + costmap_wx * cos_yaw - costmap_wy * sin_yaw;
            double global_wy = robot_y + costmap_wx * sin_yaw + costmap_wy * cos_yaw;
            
            // Convert to global map coordinates
            int gmx, gmy;
            worldToMap(global_wx, global_wy, gmx, gmy);
            
            // Update global map if within bounds
            if (gmx >= 0 && gmx < static_cast<int>(global_map_.info.width) &&
                gmy >= 0 && gmy < static_cast<int>(global_map_.info.height)) {
                // Prioritize new data over old (overwrite)
                setCost(gmx, gmy, costmap_value);
            }
        }
    }
    
    // Update timestamp
    global_map_.header.stamp = odom.header.stamp;
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
    return global_map_;
}

void MapMemoryCore::worldToMap(double wx, double wy, int& mx, int& my) const {
    mx = static_cast<int>((wx - global_map_.info.origin.position.x) / global_map_.info.resolution);
    my = static_cast<int>((wy - global_map_.info.origin.position.y) / global_map_.info.resolution);
}

void MapMemoryCore::mapToWorld(int mx, int my, double& wx, double& wy) const {
    wx = global_map_.info.origin.position.x + (mx + 0.5) * global_map_.info.resolution;
    wy = global_map_.info.origin.position.y + (my + 0.5) * global_map_.info.resolution;
}

int MapMemoryCore::getCost(int mx, int my) const {
    if (mx < 0 || mx >= static_cast<int>(global_map_.info.width) ||
        my < 0 || my >= static_cast<int>(global_map_.info.height)) {
        return -1;
    }
    return global_map_.data[my * global_map_.info.width + mx];
}

void MapMemoryCore::setCost(int mx, int my, int cost) {
    if (mx >= 0 && mx < static_cast<int>(global_map_.info.width) &&
        my >= 0 && my < static_cast<int>(global_map_.info.height)) {
        global_map_.data[my * global_map_.info.width + mx] = cost;
    }
}

double MapMemoryCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) const {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

} 
