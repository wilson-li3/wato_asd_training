#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

nav_msgs::msg::OccupancyGrid CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Initialize costmap
    std::vector<int8_t> costmap(width_ * height_, 0);
    initializeCostmap(costmap);

    // Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        
        if (range < scan->range_max && range > scan->range_min && std::isfinite(range)) {
            // Calculate Cartesian coordinates
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            
            // Convert to grid coordinates
            int x_grid, y_grid;
            convertToGrid(x, y, x_grid, y_grid);
            
            // Mark obstacle if within bounds
            if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
                markObstacle(costmap, x_grid, y_grid);
            }
        }
    }

    // Inflate obstacles
    inflateObstacles(costmap);

    // Create OccupancyGrid message
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = scan->header.stamp;
    grid_msg.header.frame_id = scan->header.frame_id;
    
    grid_msg.info.resolution = resolution_;
    grid_msg.info.width = width_;
    grid_msg.info.height = height_;
    grid_msg.info.origin.position.x = -(width_ * resolution_) / 2.0;
    grid_msg.info.origin.position.y = -(height_ * resolution_) / 2.0;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;
    
    grid_msg.data = costmap;

    return grid_msg;
}

void CostmapCore::initializeCostmap(std::vector<int8_t>& costmap) {
    // Initialize all cells to 0 (free space)
    std::fill(costmap.begin(), costmap.end(), 0);
}

void CostmapCore::convertToGrid(double x, double y, int& x_grid, int& y_grid) {
    // Convert from robot frame (meters) to grid coordinates
    // Origin is at center of grid
    x_grid = static_cast<int>((x / resolution_) + (width_ / 2.0));
    y_grid = static_cast<int>((y / resolution_) + (height_ / 2.0));
}

void CostmapCore::markObstacle(std::vector<int8_t>& costmap, int x_grid, int y_grid) {
    if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
        int index = y_grid * width_ + x_grid;
        costmap[index] = max_cost_;
    }
}

void CostmapCore::inflateObstacles(std::vector<int8_t>& costmap) {
    // Create a copy for reading
    std::vector<int8_t> costmap_copy = costmap;
    
    int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    
    // For each cell in the costmap
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = y * width_ + x;
            
            // If this cell is an obstacle
            if (costmap_copy[idx] == max_cost_) {
                // Inflate around it
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            double dist = distance(x, y, nx, ny) * resolution_;
                            
                            if (dist <= inflation_radius_) {
                                int nidx = ny * width_ + nx;
                                // Calculate cost based on distance with quadratic decay for faster falloff
                                // Normalized distance: 0 at obstacle, 1 at inflation_radius
                                double normalized_dist = dist / inflation_radius_;
                                // Quadratic decay: (1 - normalized_dist)^cost_scaling_factor
                                // This creates faster cost decay, making the visual footprint smaller
                                double cost_factor = std::pow(1.0 - normalized_dist, cost_scaling_factor_);
                                int cost = static_cast<int>(max_cost_ * cost_factor);
                                // Only assign if higher than current value
                                if (cost > costmap[nidx]) {
                                    costmap[nidx] = cost;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void CostmapCore::setInflationRadius(double radius) {
    inflation_radius_ = radius;
}

void CostmapCore::setCostScalingFactor(double factor) {
    cost_scaling_factor_ = factor;
}

double CostmapCore::distance(int x1, int y1, int x2, int y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

}