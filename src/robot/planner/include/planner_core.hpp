#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>

namespace robot
{

// 2D grid index
struct CellIndex {
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const {
        return (x == other.x && y == other.y);
    }

    bool operator!=(const CellIndex &other) const {
        return (x != other.x || y != other.y);
    }
};

// Hash function for CellIndex
struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

// Structure representing a node in the A* open set
struct AStarNode {
    CellIndex index;
    double f_score;  // f = g + h

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
        return a.f_score > b.f_score;
    }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Plan path using A* algorithm
    nav_msgs::msg::Path planPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Point& start,
        const geometry_msgs::msg::Point& goal);

    // Set parameters
    void setMinimumClearance(double clearance);
    void setExecutionBuffer(double buffer);
    void setObstacleCostThreshold(int8_t threshold);

  private:
    rclcpp::Logger logger_;

    // Parameters
    double minimum_clearance_ = 0.6;  // meters (robot_radius + safety_margin + execution_buffer)
    double execution_buffer_ = 0.15;  // meters (additional buffer for execution safety)
    int8_t obstacle_cost_threshold_ = 20;  // Cost threshold (lower = more conservative, stays further from obstacles)

    // Helper functions
    void worldToMap(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map, int& mx, int& my) const;
    void mapToWorld(int mx, int my, const nav_msgs::msg::OccupancyGrid& map, double& wx, double& wy) const;
    bool isValidCell(int mx, int my, const nav_msgs::msg::OccupancyGrid& map) const;
    bool hasMinimumClearance(int mx, int my, const nav_msgs::msg::OccupancyGrid& map) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
    std::vector<CellIndex> getNeighbors(const CellIndex& cell) const;
    double getCost(int mx, int my, const nav_msgs::msg::OccupancyGrid& map) const;
    bool validatePathClearance(const nav_msgs::msg::Path& path, const nav_msgs::msg::OccupancyGrid& map) const;
};

}  

#endif  
