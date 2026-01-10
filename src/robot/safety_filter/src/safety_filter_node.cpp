#include "safety_filter_node.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>

SafetyFilterNode::SafetyFilterNode() : Node("safety_filter") {
    // Declare and get parameters
    this->declare_parameter<double>("robot_radius", 0.25);
    this->declare_parameter<double>("safety_margin", 0.15);
    this->declare_parameter<double>("min_front", 0.0);  // 0 = auto-compute
    this->declare_parameter<double>("min_side", 0.0);
    this->declare_parameter<double>("min_back", 0.0);
    this->declare_parameter<double>("hard_stop_distance", 0.35);
    this->declare_parameter<double>("max_linear", 0.5);
    this->declare_parameter<double>("max_angular", 1.0);
    this->declare_parameter<double>("backup_duration", 3.0);
    this->declare_parameter<double>("backup_speed", -0.3);
    this->declare_parameter<double>("rotation_speed", 0.6);
    this->declare_parameter<double>("stuck_timeout", 5.0);
    this->declare_parameter<double>("recovery_timeout", 10.0);
    
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    safety_margin_ = this->get_parameter("safety_margin").as_double();
    double min_front = this->get_parameter("min_front").as_double();
    double min_side = this->get_parameter("min_side").as_double();
    double min_back = this->get_parameter("min_back").as_double();
    hard_stop_distance_ = this->get_parameter("hard_stop_distance").as_double();
    max_linear_ = this->get_parameter("max_linear").as_double();
    max_angular_ = this->get_parameter("max_angular").as_double();
    backup_duration_ = this->get_parameter("backup_duration").as_double();
    backup_speed_ = this->get_parameter("backup_speed").as_double();
    rotation_speed_ = this->get_parameter("rotation_speed").as_double();
    stuck_timeout_ = this->get_parameter("stuck_timeout").as_double();
    recovery_timeout_ = this->get_parameter("recovery_timeout").as_double();
    
    // Auto-compute min distances if not set
    min_front_ = (min_front > 0.0) ? min_front : (robot_radius_ + safety_margin_) * 1.5;
    min_side_ = (min_side > 0.0) ? min_side : (robot_radius_ + safety_margin_);
    min_back_ = (min_back > 0.0) ? min_back : (robot_radius_ + safety_margin_) * 0.8;
    
    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_planned", 10, 
        std::bind(&SafetyFilterNode::cmdVelCallback, this, std::placeholders::_1));
    
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10, 
        std::bind(&SafetyFilterNode::lidarCallback, this, std::placeholders::_1));
    
    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    safety_state_pub_ = this->create_publisher<std_msgs::msg::String>("/safety_state", 10);
    
    // Initialize state
    recovery_state_ = RecoveryState::NONE;
    last_movement_time_ = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "Safety filter node initialized:");
    RCLCPP_INFO(this->get_logger(), "  robot_radius=%.3f m", robot_radius_);
    RCLCPP_INFO(this->get_logger(), "  safety_margin=%.3f m", safety_margin_);
    RCLCPP_INFO(this->get_logger(), "  min_front=%.3f m", min_front_);
    RCLCPP_INFO(this->get_logger(), "  min_side=%.3f m", min_side_);
    RCLCPP_INFO(this->get_logger(), "  hard_stop_distance=%.3f m", hard_stop_distance_);
}

void SafetyFilterNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = *msg;
    scan_received_ = true;
}

void SafetyFilterNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!scan_received_) {
        // No scan data, pass through but warn
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No lidar scan received, passing through cmd_vel");
        cmd_vel_pub_->publish(*msg);
        return;
    }
    
    // Update recovery state machine
    updateRecoveryState(*msg, latest_scan_);
    
    // Filter velocity based on lidar scan and recovery state
    geometry_msgs::msg::Twist safe_vel = filterVelocity(*msg, latest_scan_);
    
    // Publish safe velocity
    cmd_vel_pub_->publish(safe_vel);
    
    // Publish safety state
    std_msgs::msg::String state_msg;
    switch (recovery_state_) {
        case RecoveryState::NONE:
            state_msg.data = "NORMAL";
            break;
        case RecoveryState::STOPPED:
            state_msg.data = "STOPPED";
            break;
        case RecoveryState::BACKING_UP:
            state_msg.data = "BACKING_UP";
            break;
        case RecoveryState::ROTATING:
            state_msg.data = "ROTATING";
            break;
        case RecoveryState::RESUMING:
            state_msg.data = "RESUMING";
            break;
    }
    safety_state_pub_->publish(state_msg);
    
    // Track movement for stuck detection
    // Extract position from cmd_vel would require odom, so we use time-based check instead
    auto now = this->get_clock()->now();
    if (std::abs(safe_vel.linear.x) > 0.01 || std::abs(safe_vel.angular.z) > 0.01) {
        last_movement_time_ = now;
    }
}

geometry_msgs::msg::Twist SafetyFilterNode::filterVelocity(
    const geometry_msgs::msg::Twist& cmd_vel,
    const sensor_msgs::msg::LaserScan& scan) {
    
    geometry_msgs::msg::Twist safe_vel = cmd_vel;
    
    // PRIORITY 1: Recovery state machine takes precedence
    if (recovery_state_ != RecoveryState::NONE) {
        return executeRecovery();
    }
    
    // PRIORITY 2: Hard stop checks
    double front_dist = getMinDistanceInSector(scan, -M_PI/2.0, M_PI/2.0);
    double left_dist = getMinDistanceInSector(scan, M_PI/4.0, 3.0*M_PI/4.0);
    double right_dist = getMinDistanceInSector(scan, -3.0*M_PI/4.0, -M_PI/4.0);
    double back_dist = getMinDistanceInSector(scan, 3.0*M_PI/4.0, -3.0*M_PI/4.0);
    
    // Emergency stop: anything extremely close
    double min_all = std::min({front_dist, left_dist, right_dist, back_dist});
    if (min_all < hard_stop_distance_) {
        safe_vel.linear.x = 0.0;
        safe_vel.angular.z = 0.0;
        recovery_state_ = RecoveryState::STOPPED;
        recovery_start_time_ = this->get_clock()->now();
        RCLCPP_WARN(this->get_logger(), "Hard stop: obstacle at %.2f m", min_all);
        return safe_vel;
    }
    
    // Direction-specific checks
    if (cmd_vel.linear.x > 0.0) {
        // Moving forward - check front
        if (front_dist < min_front_) {
            safe_vel.linear.x = 0.0;
            safe_vel.angular.z = 0.0;
            recovery_state_ = RecoveryState::STOPPED;
            recovery_start_time_ = this->get_clock()->now();
            RCLCPP_WARN(this->get_logger(), "Stop front: obstacle at %.2f m (required: %.2f m)", 
                       front_dist, min_front_);
            return safe_vel;
        }
        
        // Check turning clearance
        if (cmd_vel.angular.z > 0.1) {
            // Turning left - check left-front
            double left_front = getMinDistanceInSector(scan, M_PI/6.0, M_PI/2.0);
            if (left_front < min_side_ * 1.5) {
                safe_vel.linear.x *= 0.5;  // Slow down
                safe_vel.angular.z *= 0.7;  // Reduce turn rate
            }
        } else if (cmd_vel.angular.z < -0.1) {
            // Turning right - check right-front
            double right_front = getMinDistanceInSector(scan, -M_PI/2.0, -M_PI/6.0);
            if (right_front < min_side_ * 1.5) {
                safe_vel.linear.x *= 0.5;  // Slow down
                safe_vel.angular.z *= 0.7;  // Reduce turn rate
            }
        }
        
        // Check sides
        if (left_dist < min_side_ && cmd_vel.angular.z > 0.0) {
            safe_vel.angular.z = 0.0;  // Stop turning left
        }
        if (right_dist < min_side_ && cmd_vel.angular.z < 0.0) {
            safe_vel.angular.z = 0.0;  // Stop turning right
        }
    }
    
    if (cmd_vel.linear.x < 0.0) {
        // Moving backward - check back
        if (back_dist < min_back_) {
            safe_vel.linear.x = 0.0;
            safe_vel.angular.z = 0.0;
            recovery_state_ = RecoveryState::STOPPED;
            recovery_start_time_ = this->get_clock()->now();
            return safe_vel;
        }
    }
    
    // Clamp velocities to limits
    safe_vel.linear.x = std::max(-max_linear_, std::min(max_linear_, safe_vel.linear.x));
    safe_vel.angular.z = std::max(-max_angular_, std::min(max_angular_, safe_vel.angular.z));
    
    return safe_vel;
}

double SafetyFilterNode::getMinDistanceInSector(const sensor_msgs::msg::LaserScan& scan,
                                                  double min_angle, double max_angle) const {
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double angle = scan.angle_min + i * scan.angle_increment;
        
        // Normalize angle to [-pi, pi]
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        
        // Check if angle is in sector
        bool in_sector = false;
        if (min_angle <= max_angle) {
            in_sector = (angle >= min_angle && angle <= max_angle);
        } else {
            // Sector crosses -pi/pi boundary
            in_sector = (angle >= min_angle || angle <= max_angle);
        }
        
        if (in_sector) {
            double range = scan.ranges[i];
            if (range > scan.range_min && range < scan.range_max && std::isfinite(range)) {
                min_dist = std::min(min_dist, range);
            }
        }
    }
    
    return (min_dist == std::numeric_limits<double>::max()) ? scan.range_max : min_dist;
}

void SafetyFilterNode::updateRecoveryState(const geometry_msgs::msg::Twist& cmd_vel,
                                            const sensor_msgs::msg::LaserScan& scan) {
    (void)cmd_vel;  // Parameter reserved for future use
    auto now = this->get_clock()->now();
    double elapsed = (now - recovery_start_time_).seconds();
    
    // Check if stuck (no movement for timeout period)
    double time_since_movement = (now - last_movement_time_).seconds();
    bool is_stuck = (time_since_movement > stuck_timeout_);
    
    // Check clearance ahead
    double front_dist = getMinDistanceInSector(scan, -M_PI/2.0, M_PI/2.0);
    bool path_clear = (front_dist > min_front_ * 1.5);
    
    switch (recovery_state_) {
        case RecoveryState::NONE:
            // Check if stuck or hard stop needed
            if (is_stuck || front_dist < hard_stop_distance_) {
                recovery_state_ = RecoveryState::STOPPED;
                recovery_start_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Entering recovery: stuck=%d, front_dist=%.2f m",
                           is_stuck, front_dist);
            }
            break;
            
        case RecoveryState::STOPPED:
            // Start backing up
            if (elapsed > 0.5) {
                recovery_state_ = RecoveryState::BACKING_UP;
                recovery_start_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Starting backup");
            }
            break;
            
        case RecoveryState::BACKING_UP:
            // Back up for specified duration
            if (elapsed >= backup_duration_) {
                recovery_state_ = RecoveryState::ROTATING;
                recovery_start_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Backup complete, starting rotation");
            } else if (path_clear && elapsed > 1.0) {
                // Path cleared during backup
                recovery_state_ = RecoveryState::RESUMING;
                recovery_start_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Path cleared during backup");
            }
            break;
            
        case RecoveryState::ROTATING:
            // Rotate to find clear path
            if (path_clear && elapsed > 1.0) {
                recovery_state_ = RecoveryState::RESUMING;
                recovery_start_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Clear path found");
            } else if (elapsed > recovery_timeout_) {
                // Timeout, restart recovery
                recovery_state_ = RecoveryState::STOPPED;
                recovery_start_time_ = now;
                RCLCPP_WARN(this->get_logger(), "Recovery timeout, restarting");
            }
            break;
            
        case RecoveryState::RESUMING:
            // Resume normal operation
            if (elapsed > 1.0 && path_clear) {
                recovery_state_ = RecoveryState::NONE;
                RCLCPP_INFO(this->get_logger(), "Recovery complete, resuming");
            } else if (!path_clear && elapsed > 0.5) {
                // Path blocked again, restart recovery
                recovery_state_ = RecoveryState::STOPPED;
                recovery_start_time_ = now;
                RCLCPP_WARN(this->get_logger(), "Path blocked again, restarting recovery");
            }
            break;
    }
}

geometry_msgs::msg::Twist SafetyFilterNode::executeRecovery() {
    geometry_msgs::msg::Twist recovery_cmd;
    auto now = this->get_clock()->now();
    (void)now;  // May be used in future for timeout checks
    
    switch (recovery_state_) {
        case RecoveryState::STOPPED:
            recovery_cmd.linear.x = 0.0;
            recovery_cmd.angular.z = 0.0;
            break;
            
        case RecoveryState::BACKING_UP:
            // Back up straight
            recovery_cmd.linear.x = backup_speed_;
            recovery_cmd.angular.z = 0.0;
            break;
            
        case RecoveryState::ROTATING:
            // Rotate to find clear path (rotate away from closest obstacle)
            {
                double left_dist = getMinDistanceInSector(latest_scan_, M_PI/4.0, 3.0*M_PI/4.0);
                double right_dist = getMinDistanceInSector(latest_scan_, -3.0*M_PI/4.0, -M_PI/4.0);
                
                if (left_dist > right_dist + 0.2) {
                    // More clearance on left, rotate left
                    recovery_cmd.angular.z = rotation_speed_;
                } else if (right_dist > left_dist + 0.2) {
                    // More clearance on right, rotate right
                    recovery_cmd.angular.z = -rotation_speed_;
                } else {
                    // Balanced, rotate left by default
                    recovery_cmd.angular.z = rotation_speed_;
                }
                recovery_cmd.linear.x = 0.0;
            }
            break;
            
        case RecoveryState::RESUMING:
            // Gradually resume forward motion
            recovery_cmd.linear.x = max_linear_ * 0.3;
            recovery_cmd.angular.z = 0.0;
            break;
            
        case RecoveryState::NONE:
        default:
            recovery_cmd.linear.x = 0.0;
            recovery_cmd.angular.z = 0.0;
            break;
    }
    
    return recovery_cmd;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyFilterNode>());
    rclcpp::shutdown();
    return 0;
}
