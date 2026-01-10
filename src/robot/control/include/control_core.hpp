#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath>
#include <optional>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);
  
    // Compute velocity command using Pure Pursuit
    geometry_msgs::msg::Twist computeVelocity(
        const nav_msgs::msg::Path& path,
        const nav_msgs::msg::Odometry& odom);
  
  private:
    rclcpp::Logger logger_;
    
    // Parameters
    double lookahead_distance_ = 1.2;  // meters (increased from 1.0 for smoother navigation)
    double goal_tolerance_ = 0.1;      // meters
    double linear_speed_ = 0.8;        // m/s (increased from 0.5 for faster movement)
    double max_angular_speed_ = 1.0;   // rad/s
    
    // Helper functions
    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::Point& robot_pos) const;
    double computeDistance(const geometry_msgs::msg::Point& a, 
                         const geometry_msgs::msg::Point& b) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;
    bool isGoalReached(const nav_msgs::msg::Path& path,
                      const geometry_msgs::msg::Point& robot_pos) const;
};

} 

#endif 
