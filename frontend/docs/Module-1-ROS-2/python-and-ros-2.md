---
sidebar_position: 4
---

# Chapter 4: Python and ROS 2: Building Intelligent Agents

## Introduction

In the previous chapter, we explored the fundamentals of ROS 2, understanding its core communication mechanisms, including nodes, topics, services, and actions. Now, we will bridge the gap between high-level AI algorithms and the low-level control of a robot. This chapter focuses on leveraging **Python**, the language of choice for AI development, to create intelligent "agents" that seamlessly interact with a ROS 2 system. Our primary tool will be the `rclpy` library, the official and highly efficient Python client library for ROS 2. We will delve into its functionalities, demonstrate practical implementations, and explore how Python agents can orchestrate complex robotic behaviors.

![Python and ROS 2 Integration with rclpy](/img/docs%205.jpg)

## Learning Objectives

By the end of this chapter, you will be able to:

-   **Understand the Role of `rclpy`**: Explain in detail how `rclpy` enables Python code to robustly interface with ROS 2, including its architecture and benefits.
-   **Create a ROS 2 Agent**: Write a Python class that encapsulates sophisticated logic for a robotic agent, utilizing ROS 2 communication patterns.
-   **Bridge AI to ROS**: Connect complex AI algorithms or decision-making processes to ROS 2 topics and services for real-time data exchange and command execution.
-   **Develop a Goal-Oriented Agent**: Create an agent capable of receiving high-level goals, processing them, and translating them into a sequence of executable ROS 2 actions, potentially managing state.

## The `rclpy` Library: Your Gateway to ROS 2 in Python

`rclpy` is not just a wrapper; it's a powerful, idiomatic Python library designed to expose the full capabilities of ROS 2 while maintaining Python's ease of use and expressiveness. It handles the complexities of inter-process communication, message serialization/deserialization, and ROS 2 graph management, ensuring that your Python code can reliably send and receive data, call services, and manage actions within a ROS 2 graph. You can find more detailed documentation on [rclpy here](https://docs.ros.org/en/humble/p/rclpy.html) and a comprehensive [ROS 2 Python tutorial here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Writing-A-Simple-Publisher-And-Subscriber-Python.html).

### Key Functionalities Provided by `rclpy`

`rclpy` provides Python bindings for all core ROS 2 concepts. Understanding these is vital for building any ROS 2 application in Python.

| Functionality    | `rclpy` Class/Method                       | Description                                                                  |
| :--------------- | :----------------------------------------- | :--------------------------------------------------------------------------- |
| **Node**         | `rclpy.node.Node`                          | Base class for a ROS 2 node. Your agent's logic inherits from this.          |
| **Publisher**    | `node.create_publisher(msg_type, topic, qos)` | Creates an object to send messages to a topic.                               |
| **Subscriber**   | `node.create_subscription(msg_type, topic, callback, qos)` | Creates an object to receive messages from a topic.                          |
| **Service Server** | `node.create_service(srv_type, service, callback)` | Creates an object to provide a service.                                      |
| **Service Client** | `node.create_client(srv_type, service)`      | Creates an object to call a service.                                         |
| **Action Server** | `rclpy.action.ActionServer`                | Implements a server for long-running, preemptible tasks with feedback.       |
| **Action Client** | `rclpy.action.ActionClient`                | Creates a client to send goals to an action server and receive feedback/results. |
| **Timers**       | `node.create_timer(period, callback)`      | Creates a periodic callback to execute code at fixed intervals.              |
| **Logging**      | `node.get_logger().info()`, etc.           | Provides standard ROS 2 logging functionality.                               |
| **Spinning**     | `rclpy.spin(node)`, `rclpy.spin_once(node)` | Processes pending ROS 2 callbacks and events.                                |

## Building a Simple Obstacle Avoidance Agent

Let's develop a more comprehensive agent that demonstrates reactive behavior based on sensor input. This **ObstacleAvoider** agent will move a robot forward but stop or turn if an obstacle is detected within a certain range. We assume the robot subscribes to a `/scan` topic for laser scan data (of type `sensor_msgs.msg.LaserScan`) and publishes velocity commands to a `/cmd_vel` topic (of type `geometry_msgs.msg.Twist`). This agent showcases fundamental perception-action loops.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math # Used for potential angle calculations

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_agent')
        # Publisher for sending velocity commands to the robot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber for receiving laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            rclpy.qos.qos_profile_sensor_data # Use QoS profile suitable for sensor data
        )
        self.get_logger().info('ObstacleAvoider agent started. Listening for /scan data.')
        
        # Internal state variables
        self.obstacle_detected = False
        self.avoidance_distance = 0.7 # meters: how close before we react
        self.last_twist_command = Twist() # Store last command to avoid constant re-publishing

        # Timer to periodically publish velocity commands (if no immediate obstacle detected)
        self.timer = self.create_timer(0.1, self.timer_callback) # Publish every 100ms

    def scan_callback(self, msg: LaserScan):
        # A more robust check for obstacles directly in front of the robot.
        # This example assumes a 360-degree LiDAR or a frontal sector.
        # We'll check a window of ranges directly ahead.
        min_front_range = float('inf')
        
        if msg.ranges:
            # Calculate indices for the front sector (e.g., +/- 30 degrees from the robot's front)
            # This requires knowing msg.angle_min, msg.angle_increment, and len(msg.ranges)
            # For simplicity, let's assume the front is roughly in the middle, or handle wrap-around for 360 scans.
            
            # Example: checking a simple central window, assuming ranges are ordered
            # This logic needs adjustment based on your specific LiDAR's angle_min/max
            
            # Let's consider a simple window: check the first N and last N values, plus values around the center index
            # This is a simplification and should be adapted to the specific LiDAR setup
            
            # Check directly in front (simplified example)
            for i in range(0, min(30, scan_size // 12)): # first 30 readings roughly
                if msg.ranges[i] < min_front_range:
                    min_front_range = msg.ranges[i]
            for i in range(max(0, scan_size - 30), scan_size): # last 30 readings roughly
                if msg.ranges[i] < min_front_range:
                    min_front_range = msg.ranges[i]
            
            # Filter out invalid readings (inf, NaN, values outside min/max range)
            min_front_range = min([r for r in msg.ranges if msg.range_min < r < msg.range_max] + [float('inf')])
            
        # Determine if an obstacle is detected
        if min_front_range < self.avoidance_distance:
            self.obstacle_detected = True
            self.get_logger().warn(f'Obstacle detected at {min_front_range:.2f}m! Stopping.')
        else:
            self.obstacle_detected = False

    def timer_callback(self):
        twist = Twist()
        if self.obstacle_detected:
            # Stop the robot if an obstacle is detected
            twist.linear.x = 0.0
            twist.angular.z = 0.5 # Gently turn to avoid
        else:
            # Move forward if no obstacle
            twist.linear.x = 0.2 # meters per second
            twist.angular.z = 0.0

        # Only publish if the command has changed to reduce communication overhead
        if twist.linear.x != self.last_twist_command.linear.x or \
           twist.angular.z != self.last_twist_command.angular.z:
            self.publisher_.publish(twist)
            self.last_twist_command = twist
            self.get_logger().info(f'Publishing: Linear X = {twist.linear.x:.2f}, Angular Z = {twist.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoider = ObstacleAvoider()
    rclpy.spin(obstacle_avoider)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    obstacle_avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
## Summary