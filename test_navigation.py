#!/usr/bin/env python3
"""
Simple script to test navigation by publishing goal points.
Run this inside the robot container or with ROS 2 installed.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import sys
import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PointStamped, '/goal_point', 10)
        self.get_logger().info('Goal publisher node started')
        
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
    
    if len(sys.argv) == 3:
        # Use command line arguments
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        node.publish_goal(x, y)
        time.sleep(0.5)  # Give it time to publish
    else:
        # Interactive mode
        print("Goal Publisher - Interactive Mode")
        print("Enter goal coordinates (or 'q' to quit):")
        
        try:
            while True:
                user_input = input("Enter goal (x y): ").strip()
                if user_input.lower() == 'q':
                    break
                
                try:
                    parts = user_input.split()
                    if len(parts) == 2:
                        x = float(parts[0])
                        y = float(parts[1])
                        node.publish_goal(x, y)
                        print(f"Published goal: ({x}, {y})")
                    else:
                        print("Invalid input. Please enter 'x y' or 'q' to quit.")
                except ValueError:
                    print("Invalid input. Please enter numbers.")
        except KeyboardInterrupt:
            print("\nExiting...")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
