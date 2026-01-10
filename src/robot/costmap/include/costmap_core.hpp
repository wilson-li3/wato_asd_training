#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Process LaserScan and generate OccupancyGrid
    nav_msgs::msg::OccupancyGrid processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Set parameters
    void setInflationRadius(double radius);
    void setCostScalingFactor(double factor);

  private:
    rclcpp::Logger logger_;

    // Costmap parameters
    double resolution_ = 0.1;  // meters per cell
    int width_ = 200;          // cells
    int height_ = 200;         // cells
    double inflation_radius_ = 0.5;  // meters (robot_radius + safety_margin: 0.25 + 0.25)
    int max_cost_ = 100;
    double cost_scaling_factor_ = 2.0;  // Quadratic decay: higher = faster falloff

    // Helper functions
    void initializeCostmap(std::vector<int8_t>& costmap);
    void convertToGrid(double x, double y, int& x_grid, int& y_grid);
    void markObstacle(std::vector<int8_t>& costmap, int x_grid, int y_grid);
    void inflateObstacles(std::vector<int8_t>& costmap);
    double distance(int x1, int y1, int x2, int y2);
};

}  

#endif  