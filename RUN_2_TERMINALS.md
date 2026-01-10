# Running the Navigation System with 2 Terminals

## Prerequisites

First, make sure you've built the system:

```bash
./watod build robot
```

## Terminal 1: Start the System

**Open Terminal 1** and run:

```bash
cd /Users/wilsonli/wato_asd_training
./watod up
```

**Keep this terminal open!** You'll see:
- All node initialization messages
- Logs from costmap, map_memory, planner, control, safety_filter nodes
- Every 2 seconds: Robot position updates
- Any errors or warnings

**What to look for:**
- ✅ "Costmap node initialized"
- ✅ "Map memory node initialized"
- ✅ "Planner node initialized"
- ✅ "Control node initialized"
- ✅ "Safety filter node initialized"
- ✅ "Robot Position: x=..., y=..., z=..." (every 2 seconds)

**To stop:** Press `Ctrl+C` in this terminal, then run `./watod down`

---

## Terminal 2: Control and Monitor

**Open Terminal 2** (new window/tab) and run:

```bash
cd /Users/wilsonli/wato_asd_training
./watod -t robot
```

This opens a bash shell inside the robot container where you can:

### Option A: Publish a Goal from Command Line

```bash
# Publish a goal at (5.0, 5.0) in sim_world frame
ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'sim_world'}, point: {x: 5.0, y: 5.0, z: 0.0}}"
```

**What should happen:**
- In Terminal 1, you'll see: "Goal received: (5.00, 5.00)"
- "Path found with N waypoints"
- Robot starts moving!
- Every 2 seconds: Updated robot position

### Option B: Monitor Topics

```bash
# List all topics
ros2 topic list

# Monitor planned path
ros2 topic echo /path

# Monitor executed path (actual trajectory)
ros2 topic echo /executed_path

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check publishing rates
ros2 topic hz /path
ros2 topic hz /costmap
ros2 topic hz /map
```

### Option C: Check Node Status

```bash
# List all nodes
ros2 node list

# Check specific node info
ros2 node info /planner_node
ros2 node info /safety_filter_node
ros2 node info /control_node
```

---

## Using Foxglove Instead of Terminal 2

If you prefer using Foxglove (recommended):

1. **Terminal 1**: Still run `./watod up` (keep it open)
2. **Foxglove Studio**: 
   - Open browser to `http://localhost:8080` (or your configured port)
   - Connect to ROS 2 bridge
   - In the 3D panel, right-click on the grid → "Publish 2D point (/goal_point)"
   - Click anywhere on the grid to set a goal
   - Watch the robot navigate!

**In Foxglove, you'll see:**
- `/path` - Planned path (green/yellow gradient line)
- `/executed_path` - Actual robot trajectory (pink/red line)
- `/map` - Global map (costmap visualization)
- `/lidar` - Laser scan points

---

## Quick Test Sequence

### Terminal 1:
```bash
cd /Users/wilsonli/wato_asd_training
./watod up
# Wait for all nodes to initialize
```

### Terminal 2:
```bash
cd /Users/wilsonli/wato_asd_training
./watod -t robot

# Inside container, publish a goal
ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'sim_world'}, point: {x: 5.0, y: 5.0, z: 0.0}}"

# Watch the robot move!
# In Terminal 1, you should see position updates every 2 seconds
```

---

## Expected Behavior

### Terminal 1 Output:
```
[INFO] [costmap_node]: Costmap node initialized with inflation_radius=0.60
[INFO] [map_memory_node]: Map memory node initialized
[INFO] [planner_node]: Planner node initialized
[INFO] [control_node]: Control node initialized
[INFO] [safety_filter_node]: Safety filter node initialized
  Front min distance: 0.50 m
  Left min distance: 0.40 m
  Right min distance: 0.40 m
  Back min distance: 0.30 m
[INFO] [control_node]: Robot Position: x=0.000, y=0.000, z=0.000
[INFO] [control_node]: Robot Position: x=0.000, y=0.000, z=0.000
... (updates every 2 seconds)

# After publishing goal:
[INFO] [planner_node]: Goal received: (5.00, 5.00)
[INFO] [planner_node]: Path found with 23 waypoints
[INFO] [planner_node]: Published path with 23 waypoints in sim_world frame
[INFO] [control_node]: Robot Position: x=0.123, y=0.045, z=0.000
[INFO] [control_node]: Robot Position: x=0.245, y=0.089, z=0.000
... (robot is moving!)
```

### Terminal 2 (Inside Container):
```bash
# After publishing goal, check topics:
$ ros2 topic echo /path
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: sim_world
poses:
- header:
    frame_id: sim_world
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
...
```

---

## Troubleshooting

### Problem: Terminal 1 shows errors
- **Check**: Make sure Docker is running (`docker ps`)
- **Check**: Did you run `./watod build robot` first?
- **Check**: Look at the error message in Terminal 1

### Problem: Robot doesn't move after publishing goal
- **Check Terminal 1**: Do you see "Goal received"?
- **Check Terminal 1**: Do you see "Path found"?
- **In Terminal 2**: `ros2 topic echo /cmd_vel` - should see velocity commands
- **In Terminal 2**: `ros2 topic hz /path` - should see path being published

### Problem: Can't open Terminal 2 container
- **Check**: Is Terminal 1 still running `./watod up`?
- **Check**: Wait a few seconds after starting Terminal 1
- **Try**: `./watod ps` to see if containers are running

---

## Stopping Everything

**In Terminal 1:**
```bash
Press Ctrl+C
./watod down
```

**In Terminal 2:**
```bash
# Just close the terminal or type 'exit'
```

---

## Summary

- **Terminal 1**: `./watod up` - Keeps system running
- **Terminal 2**: `./watod -t robot` - Control and monitor
- **OR use Foxglove** instead of Terminal 2 for visual control

The robot will navigate to any goal you publish and maintain safe clearance from obstacles!
