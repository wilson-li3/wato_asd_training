#ifndef SAFETY_FILTER_NODE_HPP_
#define SAFETY_FILTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

class SafetyFilterNode : public rclcpp::Node {
  public:
    SafetyFilterNode();

  private:
    // Subscribers and Publishers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_state_pub_;
    
    // Callbacks
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // Safety functions
    geometry_msgs::msg::Twist filterVelocity(const geometry_msgs::msg::Twist& cmd_vel,
                                              const sensor_msgs::msg::LaserScan& scan);
    double getMinDistanceInSector(const sensor_msgs::msg::LaserScan& scan,
                                   double min_angle, double max_angle) const;
    
    // Recovery state machine
    enum class RecoveryState {
      NONE,
      STOPPED,
      BACKING_UP,
      ROTATING,
      RESUMING
    };
    
    void updateRecoveryState(const geometry_msgs::msg::Twist& cmd_vel,
                             const sensor_msgs::msg::LaserScan& scan);
    geometry_msgs::msg::Twist executeRecovery();
    
    // Data storage
    sensor_msgs::msg::LaserScan latest_scan_;
    bool scan_received_ = false;
    
    // Parameters
    double robot_radius_ = 0.25;  // meters
    double safety_margin_ = 0.15;  // meters
    double min_front_ = 0.5;  // meters
    double min_side_ = 0.4;  // meters
    double min_back_ = 0.3;  // meters
    double hard_stop_distance_ = 0.35;  // meters
    double max_linear_ = 0.5;  // m/s
    double max_angular_ = 1.0;  // rad/s
    
    // Recovery parameters
    double backup_duration_ = 3.0;  // seconds
    double backup_speed_ = -0.3;  // m/s (negative = backward)
    double rotation_speed_ = 0.6;  // rad/s
    double stuck_timeout_ = 5.0;  // seconds
    double recovery_timeout_ = 10.0;  // seconds
    
    // Recovery state
    RecoveryState recovery_state_ = RecoveryState::NONE;
    rclcpp::Time recovery_start_time_;
    rclcpp::Time last_movement_time_;
    geometry_msgs::msg::Point last_position_;
    int stuck_count_ = 0;
};

#endif
