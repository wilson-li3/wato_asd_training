# Quick Testing Reference

## 1. Build and Start

```bash
# Build the system
./watod build

# Start simulation and robot
./watod up
```

## 2. Publish a Goal (Quick Method)

Open a terminal in the robot container:

```bash
./watod -t robot
```

Then publish a goal:

```bash
# Method 1: Using ROS 2 command
ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, point: {x: 5.0, y: 5.0, z: 0.0}}"

# Method 2: Using the test script (if copied into container)
python3 test_navigation.py 5.0 5.0
```

## 3. Monitor Topics

```bash
# In robot container
./watod -t robot

# Check if topics are publishing
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /path
ros2 topic hz /costmap
```

## 4. Check Node Status

```bash
# List all nodes
ros2 node list

# Check specific node info
ros2 node info /costmap_node
ros2 node info /planner_node
ros2 node info /control_node
```

## 5. Expected Output

When working correctly, you should see:

1. **Costmap node**: Publishing `/costmap` messages
2. **Map memory node**: Publishing `/map` messages (check logs for "Initial map published")
3. **Planner node**: When goal is published, should see "Goal received" and "Path found with N waypoints"
4. **Control node**: Publishing `/cmd_vel` with non-zero values
5. **Robot**: Should start moving toward the goal

## 6. Stop Everything

```bash
./watod down
```

## Troubleshooting

- **No movement?** Check `/cmd_vel` topic
- **No path?** Check if map is being published
- **Errors?** Check node logs in the terminal where you ran `./watod up`
