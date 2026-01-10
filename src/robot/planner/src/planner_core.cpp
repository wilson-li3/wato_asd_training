#include "planner_core.hpp"
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal) {
    
    nav_msgs::msg::Path path;
    path.header.stamp = map.header.stamp;
    path.header.frame_id = map.header.frame_id;
    
    // Convert start and goal to map coordinates
    int start_mx, start_my, goal_mx, goal_my;
    worldToMap(start.x, start.y, map, start_mx, start_my);
    worldToMap(goal.x, goal.y, map, goal_mx, goal_my);
    
    CellIndex start_idx(start_mx, start_my);
    CellIndex goal_idx(goal_mx, goal_my);
    
    // Check if start and goal are valid
    if (!isValidCell(start_mx, start_my, map) || !isValidCell(goal_mx, goal_my, map)) {
        RCLCPP_WARN(logger_, "Start or goal is invalid or in obstacle");
        return path;
    }
    
    // A* algorithm
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    
    // Initialize
    g_score[start_idx] = 0.0;
    open_set.push(AStarNode(start_idx, heuristic(start_idx, goal_idx)));
    
    bool found_path = false;
    
    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();
        
        CellIndex current_idx = current.index;
        
        // Check if we reached the goal
        if (current_idx == goal_idx) {
            found_path = true;
            break;
        }
        
        // Get neighbors
        std::vector<CellIndex> neighbors = getNeighbors(current_idx);
        
        for (const auto& neighbor : neighbors) {
            // Check both validity and minimum clearance
            if (!isValidCell(neighbor.x, neighbor.y, map)) {
                continue;
            }
            
            // Calculate tentative g_score with cost penalty for proximity to obstacles
            double tentative_g = g_score[current_idx] + getCost(neighbor.x, neighbor.y, map);
            
            // If this path to neighbor is better
            if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current_idx;
                g_score[neighbor] = tentative_g;
                double f = tentative_g + heuristic(neighbor, goal_idx);
                open_set.push(AStarNode(neighbor, f));
            }
        }
    }
    
    // Reconstruct path
    if (found_path) {
        std::vector<CellIndex> path_cells;
        CellIndex current = goal_idx;
        
        while (current != start_idx) {
            path_cells.push_back(current);
            if (came_from.find(current) != came_from.end()) {
                current = came_from[current];
            } else {
                break;
            }
        }
        path_cells.push_back(start_idx);
        std::reverse(path_cells.begin(), path_cells.end());
        
        // Convert to Path message
        for (const auto& cell : path_cells) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path.header;
            mapToWorld(cell.x, cell.y, map, pose_stamped.pose.position.x, pose_stamped.pose.position.y);
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;
            path.poses.push_back(pose_stamped);
        }
        
        // Validate path clearance - ensure all waypoints maintain minimum clearance
        if (!validatePathClearance(path, map)) {
            RCLCPP_WARN(logger_, "Path found but does not meet minimum clearance requirement, rejecting");
            path.poses.clear();  // Return empty path
            return path;
        }
        
        RCLCPP_INFO(logger_, "Path found with %zu waypoints (minimum_clearance=%.2f m)", 
                    path.poses.size(), minimum_clearance_);
    } else {
        RCLCPP_WARN(logger_, "No path found from start to goal");
    }
    
    return path;
}

void PlannerCore::worldToMap(double wx, double wy, const nav_msgs::msg::OccupancyGrid& map, int& mx, int& my) const {
    mx = static_cast<int>((wx - map.info.origin.position.x) / map.info.resolution);
    my = static_cast<int>((wy - map.info.origin.position.y) / map.info.resolution);
}

void PlannerCore::mapToWorld(int mx, int my, const nav_msgs::msg::OccupancyGrid& map, double& wx, double& wy) const {
    wx = map.info.origin.position.x + (mx + 0.5) * map.info.resolution;
    wy = map.info.origin.position.y + (my + 0.5) * map.info.resolution;
}

bool PlannerCore::isValidCell(int mx, int my, const nav_msgs::msg::OccupancyGrid& map) const {
    if (mx < 0 || mx >= static_cast<int>(map.info.width) ||
        my < 0 || my >= static_cast<int>(map.info.height)) {
        return false;
    }
    
    // Use stricter threshold for planning - ensures path stays away from obstacles
    return hasMinimumClearance(mx, my, map);
}

bool PlannerCore::hasMinimumClearance(int mx, int my, const nav_msgs::msg::OccupancyGrid& map) const {
    if (mx < 0 || mx >= static_cast<int>(map.info.width) ||
        my < 0 || my >= static_cast<int>(map.info.height)) {
        return false;
    }
    
    // Check if this cell itself is an obstacle
    int idx = my * map.info.width + mx;
    int8_t cost = map.data[idx];
    if (cost >= obstacle_cost_threshold_) {
        return false;  // Cell is obstacle or too close to obstacle
    }
    
    // Check clearance in a radius around this cell
    // Convert minimum_clearance to grid cells
    int clearance_cells = static_cast<int>(std::ceil(minimum_clearance_ / map.info.resolution));
    
    // Check all cells within clearance radius
    for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
        for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
            double dist = std::sqrt(dx * dx + dy * dy) * map.info.resolution;
            if (dist <= minimum_clearance_) {
                int nx = mx + dx;
                int ny = my + dy;
                
                if (nx >= 0 && nx < static_cast<int>(map.info.width) &&
                    ny >= 0 && ny < static_cast<int>(map.info.height)) {
                    int nidx = ny * map.info.width + nx;
                    int8_t neighbor_cost = map.data[nidx];
                    
                    // If any cell within clearance radius is an obstacle, this cell is invalid
                    if (neighbor_cost >= obstacle_cost_threshold_) {
                        return false;
                    }
                }
            }
        }
    }
    
    return true;
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
    // Euclidean distance
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& cell) const {
    std::vector<CellIndex> neighbors;
    // 8-connected neighbors
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            neighbors.push_back(CellIndex(cell.x + dx, cell.y + dy));
        }
    }
    return neighbors;
}

double PlannerCore::getCost(int mx, int my, const nav_msgs::msg::OccupancyGrid& map) const {
    if (mx < 0 || mx >= static_cast<int>(map.info.width) ||
        my < 0 || my >= static_cast<int>(map.info.height)) {
        return 1000.0;  // High cost for out of bounds
    }
    
    int idx = my * map.info.width + mx;
    int8_t cost = map.data[idx];
    
    // Convert cost to distance (0-100 -> 1.0-2.0)
    // Add penalty for cells closer to obstacles to prefer paths with more clearance
    double base_cost = 1.0 + (cost / 100.0);
    
    // Additional penalty for cells that are close to obstacles (even if not obstacles themselves)
    // This encourages the planner to find paths with better clearance
    if (cost > 0) {
        double penalty = (cost / 100.0) * 2.0;  // Extra penalty for proximity to obstacles
        base_cost += penalty;
    }
    
    return base_cost;
}

bool PlannerCore::validatePathClearance(const nav_msgs::msg::Path& path, 
                                       const nav_msgs::msg::OccupancyGrid& map) const {
    // Check each waypoint in the path to ensure it maintains minimum clearance
    for (const auto& pose_stamped : path.poses) {
        int mx, my;
        worldToMap(pose_stamped.pose.position.x, pose_stamped.pose.position.y, map, mx, my);
        
        if (!hasMinimumClearance(mx, my, map)) {
            return false;  // Path waypoint violates minimum clearance
        }
    }
    return true;
}

void PlannerCore::setMinimumClearance(double clearance) {
    minimum_clearance_ = clearance;
}

void PlannerCore::setExecutionBuffer(double buffer) {
    execution_buffer_ = buffer;
}

void PlannerCore::setObstacleCostThreshold(int8_t threshold) {
    obstacle_cost_threshold_ = threshold;
}

} 
