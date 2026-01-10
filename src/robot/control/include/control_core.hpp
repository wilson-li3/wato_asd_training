#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <optional>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);
  
    // Compute velocity command using Pure Pursuit with clearance checking
    geometry_msgs::msg::Twist computeVelocity(
        const nav_msgs::msg::Path& path,
        const nav_msgs::msg::Odometry& odom,
        const sensor_msgs::msg::LaserScan* lidar = nullptr);

    // Set parameters
    void setMinimumClearance(double clearance);
    void setExecutionBuffer(double buffer);
    void setLowClearanceSlowdownFactor(double factor);
    void setLowClearanceThreshold(double threshold);
  
  private:
    rclcpp::Logger logger_;
    
    // Parameters
    double lookahead_distance_ = 1.2;  // meters
    double goal_tolerance_ = 0.1;      // meters
    double linear_speed_ = 0.8;        // m/s
    double max_angular_speed_ = 1.0;   // rad/s
    
    // Clearance parameters
    double minimum_clearance_ = 0.6;  // meters (robot_radius + safety_margin + execution_buffer)
    double execution_buffer_ = 0.15;  // meters
    double low_clearance_threshold_ = 0.8;  // meters (when to start slowing down)
    double low_clearance_slowdown_factor_ = 0.5;  // Factor to reduce speed when clearance is low
    double sharp_turn_clearance_threshold_ = 1.0;  // meters (require more clearance for sharp turns)
    double sharp_turn_angle_threshold_ = 0.5;  // rad (~29 degrees)
    
    // Helper functions
    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(
        const nav_msgs::msg::Path& path,
        const geometry_msgs::msg::Point& robot_pos) const;
    double computeDistance(const geometry_msgs::msg::Point& a, 
                         const geometry_msgs::msg::Point& b) const;
    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;
    bool isGoalReached(const nav_msgs::msg::Path& path,
                      const geometry_msgs::msg::Point& robot_pos) const;
    double getMinimumClearanceInDirection(const sensor_msgs::msg::LaserScan& lidar, 
                                         double direction_angle, 
                                         double angle_range) const;
    void applyClearanceBasedVelocityLimits(geometry_msgs::msg::Twist& cmd_vel,
                                          const sensor_msgs::msg::LaserScan& lidar,
                                          double steering_angle) const;
};

} 

#endif 
