#include "map_memory_node.hpp"
#include <chrono>
#include <cmath>

MapMemoryNode::MapMemoryNode() 
  : Node("map_memory"), 
    map_memory_(robot::MapMemoryCore(this->get_logger())),
    last_x_(0.0), 
    last_y_(0.0) {
  
  // Initialize subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  
  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  
  // Initialize timer (1 second)
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
  
  RCLCPP_INFO(this->get_logger(), "Map memory node initialized");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = *msg;
  odom_received_ = true;
  
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  
  // Compute distance traveled
  double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
  
  if (distance >= distance_threshold_) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_ && odom_received_) {
    // Integrate costmap into global map
    map_memory_.integrateCostmap(latest_costmap_, latest_odom_);
    
    // Publish the updated map
    auto global_map = map_memory_.getGlobalMap();
    map_pub_->publish(global_map);
    
    should_update_map_ = false;
    costmap_updated_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Map updated and published");
  } else if (costmap_updated_ && odom_received_) {
    // Publish initial map on first costmap (even without movement)
    // This ensures the planner has a map to work with
    map_memory_.integrateCostmap(latest_costmap_, latest_odom_);
    auto global_map = map_memory_.getGlobalMap();
    map_pub_->publish(global_map);
    costmap_updated_ = false;
    RCLCPP_INFO(this->get_logger(), "Initial map published");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
