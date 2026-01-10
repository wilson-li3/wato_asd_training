#include "control_node.hpp"
#include <chrono>

ControlNode::ControlNode() 
  : Node("control_node"), 
    control_(robot::ControlCore(this->get_logger())) {
  
  // Declare and get parameters
  this->declare_parameter<double>("robot_radius", 0.25);
  this->declare_parameter<double>("safety_margin", 0.25);
  this->declare_parameter<double>("execution_buffer", 0.15);
  this->declare_parameter<double>("low_clearance_slowdown_factor", 0.5);
  this->declare_parameter<double>("low_clearance_threshold", 0.8);
  
  double robot_radius = this->get_parameter("robot_radius").as_double();
  double safety_margin = this->get_parameter("safety_margin").as_double();
  double execution_buffer = this->get_parameter("execution_buffer").as_double();
  double slowdown_factor = this->get_parameter("low_clearance_slowdown_factor").as_double();
  double low_clearance_threshold = this->get_parameter("low_clearance_threshold").as_double();
  
  // Calculate minimum clearance: robot_radius + safety_margin + execution_buffer
  double minimum_clearance = robot_radius + safety_margin + execution_buffer;
  
  // Set control parameters
  control_.setMinimumClearance(minimum_clearance);
  control_.setExecutionBuffer(execution_buffer);
  control_.setLowClearanceSlowdownFactor(slowdown_factor);
  control_.setLowClearanceThreshold(low_clearance_threshold);
  
  // Subscribers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&ControlNode::lidarCallback, this, std::placeholders::_1));
  
  // Publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
  // Timer (10 Hz = 100ms)
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
  
  // Timer to print robot position every 2 seconds
  position_timer_ = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&ControlNode::printPosition, this));
  
  RCLCPP_INFO(this->get_logger(), "Control node initialized: minimum_clearance=%.2f m (robot=%.2f + safety=%.2f + exec_buffer=%.2f)",
              minimum_clearance, robot_radius, safety_margin, execution_buffer);
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
  path_received_ = true;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = *msg;
  odom_received_ = true;
}

void ControlNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  latest_lidar_ = *msg;
  lidar_received_ = true;
}

void ControlNode::controlLoop() {
  if (!path_received_ || !odom_received_) {
    return;
  }
  
  // Compute velocity command using Pure Pursuit with clearance checking
  const sensor_msgs::msg::LaserScan* lidar_ptr = lidar_received_ ? &latest_lidar_ : nullptr;
  auto cmd_vel = control_.computeVelocity(current_path_, robot_odom_, lidar_ptr);
  
  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}

void ControlNode::printPosition() {
  if (odom_received_) {
    double x = robot_odom_.pose.pose.position.x;
    double y = robot_odom_.pose.pose.position.y;
    double z = robot_odom_.pose.pose.position.z;
    RCLCPP_INFO(this->get_logger(), "Robot Position: x=%.3f, y=%.3f, z=%.3f", x, y, z);
  } else {
    RCLCPP_WARN(this->get_logger(), "Robot position unknown (no odometry received)");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
