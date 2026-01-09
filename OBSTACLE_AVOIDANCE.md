# Obstacle Avoidance Explanation

## Will the Robot Hit Obstacles?

**Short answer: No, the robot should avoid obstacles automatically.**

Here's how the obstacle avoidance system works:

## How Obstacle Avoidance Works

### 1. **Costmap Node** - Detects Obstacles
- Subscribes to `/lidar` (laser scanner)
- Converts laser scan data into an occupancy grid
- **Marks obstacles** with high cost values (100 = definitely occupied)
- **Inflates obstacles** with a configurable radius (default: 1.0 meter)
  - This creates a "safety buffer" around obstacles
  - Cells near obstacles get intermediate cost values based on distance

### 2. **Map Memory Node** - Builds Global Map
- Aggregates costmaps over time as the robot moves
- Creates a global map showing all detected obstacles
- Updates the map when robot moves 1.5 meters

### 3. **Planner Node** - Plans Around Obstacles
- Uses **A* pathfinding algorithm** on the occupancy grid
- **Only considers cells with cost < 50 as traversable**
- This means:
  - Cells with cost 0-49: Free space (robot can go here)
  - Cells with cost 50-100: Obstacles or inflated areas (robot avoids these)
  - Cells with cost -1: Unknown (treated as obstacles for safety)

### 4. **Control Node** - Follows Safe Path
- Uses Pure Pursuit to follow the planned path
- The path is already obstacle-free (planned by A*)
- Robot follows waypoints that avoid all obstacles

## Safety Features

1. **Inflation Radius**: Obstacles are inflated by 1.0 meter, creating a safety buffer
2. **Cost Threshold**: Planner only uses cells with cost < 50, avoiding even partially occupied cells
3. **Replanning**: If the map updates and shows new obstacles, the planner automatically replans

## What Could Go Wrong?

The robot might still hit obstacles if:

1. **Obstacle appears too quickly**: If an obstacle appears in the robot's path after planning but before the robot can stop/replan
2. **Sensor failure**: If lidar stops working, no new obstacles will be detected
3. **Very small obstacles**: Objects smaller than the costmap resolution (0.1m) might be missed
4. **Dynamic obstacles**: The current system is designed for static obstacles. Moving obstacles aren't handled

## Testing Obstacle Avoidance

To test if obstacle avoidance works:

1. **Start the simulation** with obstacles in the environment
2. **Publish a goal** that would require going through/around obstacles
3. **Monitor the path**: Check `/path` topic - it should go around obstacles, not through them
4. **Watch the robot**: It should navigate around obstacles smoothly

## Improving Obstacle Avoidance

If you want to make it safer:

1. **Increase inflation radius** in `costmap_core.cpp`:
   ```cpp
   double inflation_radius_ = 1.5;  // Increase from 1.0 to 1.5 meters
   ```

2. **Lower cost threshold** in `planner_core.cpp`:
   ```cpp
   return cost < 30;  // More conservative (was 50)
   ```

3. **Increase costmap resolution** for better obstacle detection:
   ```cpp
   double resolution_ = 0.05;  // Finer resolution (was 0.1)
   ```

## Current Configuration

- **Costmap resolution**: 0.1 meters per cell
- **Inflation radius**: 1.0 meter
- **Obstacle cost threshold**: 50 (cells with cost >= 50 are avoided)
- **Map update distance**: 1.5 meters

These settings provide a good balance between safety and efficiency for most scenarios.
