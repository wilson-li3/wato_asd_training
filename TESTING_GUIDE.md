# Testing Guide for Navigation System

This guide will help you test the navigation system to verify it's working correctly.

## Prerequisites

1. Make sure Docker is installed and running
2. Ensure you're in the project root directory

## Step 1: Build the System

Build all the packages using the watod script:

```bash
./watod build
```

This will compile all the ROS 2 nodes inside Docker containers.

## Step 2: Start the Simulation

Start the Gazebo simulation and robot nodes:

```bash
./watod up
```

This will start:
- Gazebo simulation with the robot
- All navigation nodes (costmap, map_memory, planner, control)
- Odometry spoof node

## Step 3: Monitor the System (Recommended: Use Foxglove)

### Option A: Using Foxglove Studio (Recommended)

1. **Start Foxglove** (if using the docker container):
   ```bash
   ./watod up  # Make sure vis_tools is in ACTIVE_MODULES
   ```

2. **Connect to ROS 2**:
   - Open Foxglove Studio in your browser (usually at `http://localhost:8080`)
   - Add a "ROS 2" connection
   - Use the connection string (check your docker-compose config)

3. **Add Panels**:
   - **3D Panel**: Visualize the robot, map, path, and obstacles
   - **Raw Messages**: View `/lidar`, `/costmap`, `/map`, `/path`, `/cmd_vel`
   - **Plot**: Monitor robot position, velocity, etc.

### Option B: Using ROS 2 Command Line Tools

You can also monitor topics using command line:

```bash
# Open a terminal in the robot container
./watod -t robot

# List all topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /lidar
ros2 topic echo /costmap
ros2 topic echo /map
ros2 topic echo /path
ros2 topic echo /cmd_vel
ros2 topic echo /odom/filtered
```

## Step 4: Publish a Goal Point

To test navigation, you need to publish a goal point to the `/goal_point` topic.

### Option A: Using ROS 2 Command Line

Open a terminal in the robot container:

```bash
./watod -t robot
```

Then publish a goal point:

```bash
# Publish a goal point at (x=5.0, y=5.0) in the map frame
ros2 topic pub --once /goal_point geometry_msgs/msg/PointStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, point: {x: 5.0, y: 5.0, z: 0.0}}"
```

### Option B: Using Python Script

Create a test script to publish goals:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PointStamped, '/goal_point', 10)
        
    def publish_goal(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published goal: ({x}, {y})')

def main():
    rclpy.init()
    node = GoalPublisher()
    
    # Example: publish goal at (5.0, 5.0)
    node.publish_goal(5.0, 5.0)
    
    rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 5: What to Look For

### Expected Behavior:

1. **Costmap Node**:
   - Should be receiving `/lidar` messages
   - Should be publishing `/costmap` messages
   - Check logs: `Initial map published` or `Map updated and published`

2. **Map Memory Node**:
   - Should be receiving `/costmap` and `/odom/filtered` messages
   - Should be publishing `/map` messages
   - The map should grow as the robot moves

3. **Planner Node**:
   - When you publish a goal, should see: `Goal received: (x, y)`
   - Should publish `/path` messages with waypoints
   - Check logs: `Path found with N waypoints`

4. **Control Node**:
   - Should be receiving `/path` and `/odom/filtered` messages
   - Should be publishing `/cmd_vel` messages
   - Robot should start moving toward the goal

5. **Robot Movement**:
   - Robot should follow the planned path
   - Robot should avoid obstacles (if any)
   - Robot should stop when it reaches the goal

### Debugging Tips:

1. **Check if topics are being published**:
   ```bash
   ros2 topic hz /costmap
   ros2 topic hz /map
   ros2 topic hz /path
   ros2 topic hz /cmd_vel
   ```

2. **View node logs**:
   ```bash
   # In the robot container
   ros2 node list
   ros2 node info /costmap_node
   ros2 node info /planner_node
   ```

3. **Check if robot is receiving commands**:
   ```bash
   ros2 topic echo /cmd_vel
   # You should see linear.x and angular.z values
   ```

4. **Verify coordinate frames**:
   - Make sure all messages use the correct frame_id (usually 'map' or 'base_link')
   - Check TF tree: `ros2 run tf2_tools view_frames`

## Step 6: Test Different Scenarios

1. **Simple Navigation**:
   - Publish a goal 2-3 meters away
   - Robot should navigate directly to it

2. **Obstacle Avoidance**:
   - If there are obstacles in the simulation
   - Robot should plan around them

3. **Multiple Goals**:
   - Publish a goal, wait for robot to reach it
   - Publish another goal
   - Robot should plan a new path

4. **Invalid Goals**:
   - Try publishing a goal inside an obstacle
   - Planner should warn: "Start or goal is invalid or in obstacle"

## Common Issues and Solutions

### Issue: Robot doesn't move
- **Check**: Is `/cmd_vel` being published?
- **Check**: Are there any errors in the node logs?
- **Check**: Is the path being generated?

### Issue: No path found
- **Check**: Is the map being published?
- **Check**: Are start/goal positions valid (not in obstacles)?
- **Check**: Are coordinates in the correct frame?

### Issue: Robot moves but doesn't reach goal
- **Check**: Goal tolerance might be too small
- **Check**: Path might be getting blocked
- **Check**: Control parameters (lookahead distance, speed)

### Issue: Map is empty
- **Check**: Is `/lidar` topic publishing?
- **Check**: Is costmap being generated?
- **Check**: Map memory node logs

## Stopping the System

To stop everything:

```bash
./watod down
```

This will stop and remove all containers.

## Next Steps

Once basic navigation works:
- Tune control parameters (lookahead distance, speed)
- Adjust costmap resolution and inflation radius
- Test with different goal positions
- Monitor performance and optimize
