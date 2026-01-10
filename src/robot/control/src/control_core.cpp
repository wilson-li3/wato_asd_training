#include "control_core.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <limits>
#include <chrono>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const nav_msgs::msg::Path& path,
    const nav_msgs::msg::Odometry& odom,
    const sensor_msgs::msg::LaserScan* lidar) {
    
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
    
    // Calculate angle to lookahead point
    double angle_to_lookahead = std::atan2(dy, dx);
    
    // Calculate steering angle (difference between desired heading and current heading)
    double steering_angle = angle_to_lookahead - robot_yaw;
    
    // Normalize steering angle to [-pi, pi]
    while (steering_angle > M_PI) steering_angle -= 2.0 * M_PI;
    while (steering_angle < -M_PI) steering_angle += 2.0 * M_PI;
    
    // Pure Pursuit: calculate curvature
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = (2.0 * linear_speed_ * std::sin(steering_angle)) / lookahead_distance_;
    
    // Apply clearance-based velocity limits if lidar is available
    if (lidar != nullptr) {
        applyClearanceBasedVelocityLimits(cmd_vel, *lidar, steering_angle);
    }
    
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

double ControlCore::getMinimumClearanceInDirection(const sensor_msgs::msg::LaserScan& lidar,
                                                   double direction_angle,
                                                   double angle_range) const {
    double min_dist = std::numeric_limits<double>::max();
    
    // direction_angle is relative to robot front (0 = forward, positive = left, negative = right)
    // Lidar typically has: angle_min = -PI (back right), 0 = forward, angle_max = +PI (back left)
    
    // Normalize direction_angle to [-PI, PI] range (lidar frame)
    while (direction_angle > M_PI) direction_angle -= 2.0 * M_PI;
    while (direction_angle < -M_PI) direction_angle += 2.0 * M_PI;
    
    double lidar_start = direction_angle - angle_range / 2.0;
    double lidar_end = direction_angle + angle_range / 2.0;
    
    // Check each scan point
    for (size_t i = 0; i < lidar.ranges.size(); ++i) {
        double scan_angle = lidar.angle_min + i * lidar.angle_increment;
        
        // Check if this scan point is in our direction range
        bool in_range = false;
        if (lidar_start <= lidar_end) {
            in_range = (scan_angle >= lidar_start && scan_angle <= lidar_end);
        } else {
            // Handle wrap-around case (crosses -PI/PI boundary)
            in_range = (scan_angle >= lidar_start || scan_angle <= lidar_end);
        }
        
        if (in_range) {
            double range = lidar.ranges[i];
            if (std::isfinite(range) && range > lidar.range_min && range < lidar.range_max) {
                if (range < min_dist) {
                    min_dist = range;
                }
            }
        }
    }
    
    return min_dist;
}

void ControlCore::applyClearanceBasedVelocityLimits(geometry_msgs::msg::Twist& cmd_vel,
                                                    const sensor_msgs::msg::LaserScan& lidar,
                                                    double steering_angle) const {
    // Check clearance in forward direction (primary concern for linear motion)
    double forward_clearance = getMinimumClearanceInDirection(lidar, 0.0, M_PI / 4.0);  // 45 deg cone forward
    
    if (forward_clearance == std::numeric_limits<double>::max()) {
        // No valid scan data in forward direction, be conservative
        cmd_vel.linear.x *= 0.5;
    } else {
        // Check if forward clearance is below threshold - slow down significantly
        if (forward_clearance < minimum_clearance_) {
            // Emergency: very close to obstacle ahead, reduce speed dramatically
            double reduction_factor = std::max(0.1, forward_clearance / minimum_clearance_);
            cmd_vel.linear.x *= reduction_factor;
            static auto last_warn_time = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count();
            if (elapsed > 1000) {
                RCLCPP_WARN(logger_,
                            "Low forward clearance: %.2f m (required: %.2f m), reducing speed",
                            forward_clearance, minimum_clearance_);
                last_warn_time = now;
            }
        } else if (forward_clearance < low_clearance_threshold_) {
            // Warning: getting close ahead, slow down moderately
            double reduction = (low_clearance_threshold_ - forward_clearance) / 
                              (low_clearance_threshold_ - minimum_clearance_);
            double reduction_factor = 1.0 - (reduction * (1.0 - low_clearance_slowdown_factor_));
            cmd_vel.linear.x *= reduction_factor;
        }
    }
    
    // For sharp turns, check clearance on the turning side
    double abs_steering = std::abs(steering_angle);
    if (abs_steering > sharp_turn_angle_threshold_) {
        // Check clearance on the turning side (left or right)
        double turn_side_angle = (steering_angle > 0) ? M_PI / 2.0 : -M_PI / 2.0;  // Left (+90°) or Right (-90°)
        double side_clearance = getMinimumClearanceInDirection(lidar, turn_side_angle, M_PI / 3.0);  // 60 deg cone
        
        if (side_clearance != std::numeric_limits<double>::max() && 
            side_clearance < sharp_turn_clearance_threshold_) {
            // Low clearance on turning side - reduce angular velocity and linear speed
            double turn_reduction = side_clearance / sharp_turn_clearance_threshold_;
            cmd_vel.angular.z *= turn_reduction;
            // Reduce linear speed to allow more controlled turning
            cmd_vel.linear.x *= 0.7;
            static auto last_debug_time = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_debug_time).count();
            if (elapsed > 2000) {
                RCLCPP_DEBUG(logger_,
                             "Low side clearance during turn: %.2f m, reducing turn rate",
                             side_clearance);
                last_debug_time = now;
            }
        }
    }
    
    // Check side clearance even for gentle turns to avoid wall scraping
    double side_check_angle = (steering_angle > 0) ? M_PI / 3.0 : -M_PI / 3.0;  // 60° left or right
    double side_clearance = getMinimumClearanceInDirection(lidar, side_check_angle, M_PI / 6.0);
    
    if (side_clearance != std::numeric_limits<double>::max() && 
        side_clearance < minimum_clearance_ * 1.2) {  // Slightly more lenient threshold for sides
        // Side is getting close, slow down a bit
        double side_reduction = std::max(0.8, side_clearance / (minimum_clearance_ * 1.2));
        cmd_vel.linear.x *= side_reduction;
    }
    
    // Ensure minimum speed for controllability (but allow zero if too close)
    if (cmd_vel.linear.x > 0.0 && cmd_vel.linear.x < 0.05) {
        cmd_vel.linear.x = 0.0;  // Stop if speed is too low
    }
}

void ControlCore::setMinimumClearance(double clearance) {
    minimum_clearance_ = clearance;
}

void ControlCore::setExecutionBuffer(double buffer) {
    execution_buffer_ = buffer;
}

void ControlCore::setLowClearanceSlowdownFactor(double factor) {
    low_clearance_slowdown_factor_ = factor;
}

void ControlCore::setLowClearanceThreshold(double threshold) {
    low_clearance_threshold_ = threshold;
}

}  
