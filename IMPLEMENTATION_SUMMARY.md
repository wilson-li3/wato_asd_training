# A* Planner Implementation Summary

This document describes the A* path planner implementation following the exact specification. The planner integrates with Foxglove's "Publish 2D point (/goal_point)" feature to drive robot navigation.

## Overview

The planner node implements A* path planning on an occupancy grid with robot footprint inflation. It follows a strict state machine with two states: `WAITING_FOR_GOAL` and `WAITING_FOR_ROBOT_TO_REACH_GOAL`.

## Architecture

```
/goal_point (PointStamped) → planner_node
/map (OccupancyGrid)        → planner_node
/odom/filtered (Odometry)   → planner_node
                                  ↓
                              /path (Path)
```

## State Machine

### WAITING_FOR_GOAL
- Do nothing until a new `/goal_point` arrives
- No planning or publishing

### WAITING_FOR_ROBOT_TO_REACH_GOAL
- Monitor progress to goal using odometry
- Replan on:
  - Map updates
  - Timer-based checks (every 2 seconds)
  - Timeout/no-progress conditions
- Transition back to `WAITING_FOR_GOAL` when:
  - Goal reached (distance < goal_tolerance_m)
  - Timeout reached (default 60 seconds)

## Files Modified

### Planner Package (`src/robot/planner/`)

**Files rewritten:**
- `include/planner_core.hpp` - A* planner core with CellIndex, AStarNode, CompareF structs
- `src/planner_core.cpp` - A* implementation with 8-connected neighbors, inflation, Euclidean heuristic
- `include/planner_node.hpp` - Planner ROS 2 node with state machine (WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL)
- `src/planner_node.cpp` - Node logic with mapCallback, goalCallback, odomCallback, timerCallback
- `config/params.yaml` - Planner parameters (robot_radius_m, inflation_radius_m, lethal_cost, etc.)

**Key implementation details:**

1. **A* Algorithm:**
   - Open set: `priority_queue<AStarNode, vector<AStarNode>, CompareF>` (min-heap by f_score)
   - Closed set: `unordered_map<CellIndex, bool, CellIndexHash>`
   - came_from: `unordered_map<CellIndex, CellIndex, CellIndexHash>` for path reconstruction
   - g_score: `unordered_map<CellIndex, double, CellIndexHash>` for path cost from start
   - Heuristic: Euclidean distance
   - Neighbors: 8-connected (including diagonals)
   - Move cost: 1.0 for straight, sqrt(2) for diagonal + cell cost penalty

2. **Inflation:**
   - Inflates obstacles by `inflation_radius_m` parameter
   - Any cell within `inflation_radius_m` of an occupied cell (cost >= 50) is marked as blocked
   - Ensures A* only returns traversable corridors

3. **Frame Usage:**
   - `/path.header.frame_id = current_map_.header.frame_id` (uses map's frame, typically "sim_world")
   - Goal point conversion: assumes goal is in same frame as map (or uses map frame)

4. **Path Downsampling:**
   - Keeps first, last, and every 3rd waypoint to reduce number of points
   - Ensures path remains valid and navigable

5. **Callbacks (per spec):**
   - `mapCallback`: Store map; if state is `WAITING_FOR_ROBOT_TO_REACH_GOAL`, replan
   - `goalCallback`: Store goal; transition to `WAITING_FOR_ROBOT_TO_REACH_GOAL`; call `planPath()`
   - `odomCallback`: Store robot pose
   - `timerCallback` (500ms): If in `WAITING_FOR_ROBOT_TO_REACH_GOAL`:
     - If `goalReached()`: transition to `WAITING_FOR_GOAL`
     - Else if timeout/no-progress: call `planPath()`

## Parameters

### Planner (`planner/config/params.yaml`)
```yaml
robot_radius_m: 0.25      # meters, robot footprint radius (default 0.25-0.35)
inflation_radius_m: 0.5   # meters, inflation radius (default 0.5-0.7)
lethal_cost: 50           # cost threshold for occupied cells (>= 50)
goal_tolerance_m: 0.5     # meters, distance to goal to consider reached
timeout_seconds: 60.0     # seconds, timeout for reaching goal
progress_check_interval: 2.0  # seconds, interval for replanning/progress checks
```

## Build and Test

### Build
```bash
./watod build robot
./watod up robot
```

### Test Navigation
```bash
# Set a goal using Foxglove "Publish 2D point (/goal_point)" or:
ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'sim_world'}, point: {x: 5.0, y: 5.0, z: 0.0}}"
```

### Monitor Topics
```bash
# Check path planning
ros2 topic echo /path

# Check goal reception
ros2 topic echo /goal_point

# Check map
ros2 topic echo /map --no-arr
```

## Expected Behavior

1. **Initial State:** Node starts in `WAITING_FOR_GOAL`, does nothing
2. **Goal Received:** On `/goal_point`, transitions to `WAITING_FOR_ROBOT_TO_REACH_GOAL`, plans path, publishes `/path`
3. **Path Planning:** A* finds path avoiding obstacles (cost >= 50) and inflated areas
4. **Replanning:** 
   - Automatic replan every 2 seconds while waiting for robot to reach goal
   - Immediate replan when map updates
5. **Goal Reached:** When robot within 0.5m of goal, transitions back to `WAITING_FOR_GOAL`
6. **Timeout:** If goal not reached within 60 seconds, transitions back to `WAITING_FOR_GOAL`

## Dependencies

- `rclcpp` - ROS 2 C++ client library
- `nav_msgs` - Navigation messages (Path, OccupancyGrid, Odometry)
- `geometry_msgs` - Geometry messages (PointStamped)
- `std_msgs` - Standard messages

No external libraries beyond standard C++ and ROS 2.
