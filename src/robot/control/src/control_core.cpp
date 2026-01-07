#include "control_core.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const nav_msgs::msg::Path& path,
    const nav_msgs::msg::Odometry& odom) {
    
    geometry_msgs::msg::Twist cmd_vel;
    
    if (path.poses.empty()) {
        return cmd_vel;  // Return zero velocity
    }
    
    // Check if goal is reached
    geometry_msgs::msg::Point robot_pos = odom.pose.pose.position;
    if (isGoalReached(path, robot_pos)) {
        RCLCPP_INFO(logger_, "Goal reached, stopping robot");
        return cmd_vel;  // Return zero velocity
    }
    
    // Find lookahead point
    auto lookahead_opt = findLookaheadPoint(path, robot_pos);
    
    if (!lookahead_opt.has_value()) {
        // If no lookahead point found, try to go to the last point
        if (!path.poses.empty()) {
            lookahead_opt = path.poses.back().pose.position;
        } else {
            return cmd_vel;
        }
    }
    
    geometry_msgs::msg::Point lookahead = lookahead_opt.value();
    
    // Get robot orientation
    double robot_yaw = extractYaw(odom.pose.pose.orientation);
    
    // Calculate vector from robot to lookahead point
    double dx = lookahead.x - robot_pos.x;
    double dy = lookahead.y - robot_pos.y;
    double distance_to_lookahead = std::sqrt(dx * dx + dy * dy);
    
    // Calculate angle to lookahead point
    double angle_to_lookahead = std::atan2(dy, dx);
    
    // Calculate steering angle (difference between desired heading and current heading)
    double steering_angle = angle_to_lookahead - robot_yaw;
    
    // Normalize steering angle to [-pi, pi]
    while (steering_angle > M_PI) steering_angle -= 2.0 * M_PI;
    while (steering_angle < -M_PI) steering_angle += 2.0 * M_PI;
    
    // Pure Pursuit: calculate curvature
    // For differential drive: angular_velocity = (2 * linear_velocity * sin(steering_angle)) / lookahead_distance
    // Simplified: angular_velocity = linear_velocity * steering_angle / lookahead_distance
    
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = (2.0 * linear_speed_ * std::sin(steering_angle)) / lookahead_distance_;
    
    // Limit angular velocity
    if (cmd_vel.angular.z > max_angular_speed_) {
        cmd_vel.angular.z = max_angular_speed_;
    } else if (cmd_vel.angular.z < -max_angular_speed_) {
        cmd_vel.angular.z = -max_angular_speed_;
    }
    
    return cmd_vel;
}

std::optional<geometry_msgs::msg::Point> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Point& robot_pos) const {
    
    if (path.poses.empty()) {
        return std::nullopt;
    }
    
    // Find the closest point on the path to the robot
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
        double dist = computeDistance(robot_pos, path.poses[i].pose.position);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    
    // Look ahead from the closest point
    for (size_t i = closest_idx; i < path.poses.size(); ++i) {
        double dist = computeDistance(robot_pos, path.poses[i].pose.position);
        if (dist >= lookahead_distance_) {
            return path.poses[i].pose.position;
        }
    }
    
    // If no point is at lookahead distance, return the last point
    return path.poses.back().pose.position;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, 
                                    const geometry_msgs::msg::Point& b) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) const {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

bool ControlCore::isGoalReached(const nav_msgs::msg::Path& path,
                               const geometry_msgs::msg::Point& robot_pos) const {
    if (path.poses.empty()) {
        return false;
    }
    
    geometry_msgs::msg::Point goal = path.poses.back().pose.position;
    double dist = computeDistance(robot_pos, goal);
    return dist < goal_tolerance_;
}

}  
