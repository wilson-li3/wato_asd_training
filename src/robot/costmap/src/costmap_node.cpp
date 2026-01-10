#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Declare and get parameters
  this->declare_parameter<double>("inflation_radius", 0.5);
  this->declare_parameter<double>("cost_scaling_factor", 2.5);
  
  double inflation_radius = this->get_parameter("inflation_radius").as_double();
  double cost_scaling = this->get_parameter("cost_scaling_factor").as_double();
  
  // Set costmap parameters
  costmap_.setInflationRadius(inflation_radius);
  costmap_.setCostScalingFactor(cost_scaling);
  
  // Initialize subscriber for laser scan
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  
  // Initialize publisher for costmap
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  
  RCLCPP_INFO(this->get_logger(), "Costmap node initialized: inflation_radius=%.2f m, cost_scaling_factor=%.2f",
              inflation_radius, cost_scaling);
}
 
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Process laser scan and generate costmap
  auto costmap = costmap_.processLaserScan(msg);
  
  // Publish the costmap
  costmap_pub_->publish(costmap);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}