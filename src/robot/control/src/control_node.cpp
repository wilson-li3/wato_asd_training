#include "control_node.hpp"
#include <chrono>

ControlNode::ControlNode() 
  : Node("control"), 
    control_(robot::ControlCore(this->get_logger())) {
  
  // Subscribers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  
  // Publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
  // Timer (10 Hz = 100ms)
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
  
  // Timer to print robot position every 2 seconds
  position_timer_ = this->create_wall_timer(
    std::chrono::seconds(2), std::bind(&ControlNode::printPosition, this));
  
  RCLCPP_INFO(this->get_logger(), "Control node initialized");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
  path_received_ = true;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = *msg;
  odom_received_ = true;
}

void ControlNode::controlLoop() {
  if (!path_received_ || !odom_received_) {
    return;
  }
  
  // Compute velocity command using Pure Pursuit
  auto cmd_vel = control_.computeVelocity(current_path_, robot_odom_);
  
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
