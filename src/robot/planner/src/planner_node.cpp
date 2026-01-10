#include "planner_node.hpp"
#include <chrono>
#include <cmath>

PlannerNode::PlannerNode() 
  : Node("planner_node"), 
    planner_(robot::PlannerCore(this->get_logger())),
    state_(State::WAITING_FOR_GOAL) {
  
  // Declare and get parameters
  this->declare_parameter<double>("robot_radius", 0.25);
  this->declare_parameter<double>("safety_margin", 0.25);
  this->declare_parameter<double>("execution_buffer", 0.15);
  this->declare_parameter<int>("obstacle_cost_threshold", 20);
  
  double robot_radius = this->get_parameter("robot_radius").as_double();
  double safety_margin = this->get_parameter("safety_margin").as_double();
  double execution_buffer = this->get_parameter("execution_buffer").as_double();
  int obstacle_threshold = this->get_parameter("obstacle_cost_threshold").as_int();
  
  // Calculate minimum clearance: robot_radius + safety_margin + execution_buffer
  double minimum_clearance = robot_radius + safety_margin + execution_buffer;
  
  // Set planner parameters
  planner_.setMinimumClearance(minimum_clearance);
  planner_.setExecutionBuffer(execution_buffer);
  planner_.setObstacleCostThreshold(static_cast<int8_t>(obstacle_threshold));
  
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  
  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  
  // Timer (500ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "Planner node initialized: minimum_clearance=%.2f m (robot=%.2f + safety=%.2f + exec_buffer=%.2f), obstacle_threshold=%d",
              minimum_clearance, robot_radius, safety_margin, execution_buffer, obstacle_threshold);
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
  
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  goal_time_ = this->get_clock()->now();
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "Goal received: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      goal_received_ = false;
    } else {
      // Check timeout
      auto elapsed = (this->get_clock()->now() - goal_time_).seconds();
      if (elapsed > timeout_seconds_) {
        RCLCPP_WARN(this->get_logger(), "Timeout reached, replanning...");
        planPath();
        goal_time_ = this->get_clock()->now();  // Reset timer
      } else {
        // Replan periodically
        planPath();
      }
    }
  }
}

void PlannerNode::planPath() {
  if (!goal_received_ || !map_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }
  
  geometry_msgs::msg::Point start;
  start.x = robot_pose_.position.x;
  start.y = robot_pose_.position.y;
  start.z = 0.0;
  
  nav_msgs::msg::Path path = planner_.planPath(current_map_, start, goal_.point);
  
  if (!path.poses.empty()) {
    path_pub_->publish(path);
  }
}

bool PlannerNode::goalReached() const {
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  return distance < goal_tolerance_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
