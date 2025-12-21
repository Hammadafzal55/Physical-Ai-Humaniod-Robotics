---
sidebar_position: 3
---

# Chapter 3: ROS 2 Fundamentals

## Introduction

Welcome to the heart of modern robotics development. In this chapter, we will dive deep into the **Robotic Operating System (ROS 2)**. ROS is not a traditional operating system, but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. We will explore the core concepts in-depth and provide practical examples to get you started.

![ROS 2 Architecture Diagram](/img/docs%206.png)

## Learning Objectives

By the end of this chapter, you will be able to:

-   **Understand the ROS 2 Architecture**: Explain the concepts of nodes, topics, services, and actions in detail, including their underlying communication patterns.
-   **Grasp Quality of Service (QoS)**: Comprehend the importance of QoS settings in ROS 2 communication.
-   **Create and Run ROS 2 Nodes**: Write your own ROS 2 nodes in Python and understand their lifecycle.
-   **Use ROS 2 Command-Line Tools**: Effectively inspect, debug, and interact with a running ROS 2 system using various command-line utilities.
-   **Build a Multi-Node System**: Design and implement a system with multiple nodes communicating via topics and services, potentially using launch files.

## The ROS 2 Graph: A Deep Dive into Distributed Robotics

A running ROS 2 system is a graph of interconnected **nodes**. This distributed architecture is key to building complex, fault-tolerant robotic applications.

### Nodes: The Modular Building Blocks

A **node** is the fundamental unit of computation in ROS 2. Each node should be responsible for a single, well-defined, and encapsulated task. This modularity is a cornerstone of good ROS 2 design. For example, in a mobile robot:

-   A `lidar_driver` node reads raw data from the LiDAR sensor.
-   An `odometry_publisher` node estimates the robot's position and orientation.
-   A `map_server` node provides the environment map.
-   A `navigation_planner` node computes paths based on the map and goal.
-   A `motor_controller` node sends commands to the robot's wheels.

This granular approach enhances reusability, simplifies debugging, and allows for distributed development across teams or even different computing units. You can learn more about [ROS 2 Nodes here](https://docs.ros.org/en/humble/Concepts/Basic/Concepts-Nodes.html).

### Topics: Asynchronous Data Streams

**Topics** are the primary mechanism for **asynchronous, one-to-many communication** in ROS 2. They form data streams where information is continuously published by one or more nodes and subscribed to by zero or more nodes.

-   **Asynchronous**: Publishers and subscribers operate independently. They do not need to be synchronized in time or execution. A publisher can send data even if no subscriber is listening, and a subscriber can listen for data even if no publisher is active.
-   **One-to-many**: A single publisher can broadcast data to multiple subscribers, and a single subscriber can receive data from multiple publishers (though this is less common).
-   **Messages**: Data exchanged over topics are structured messages, defined using ROS Interface Definition Language (IDL) files (e.g., `.msg` files). These messages are language-agnostic.

**Real-World Analogy**: Think of topics like a live news broadcast. The news station (publisher) continuously broadcasts news (messages) on various channels (topics). Many people (subscribers) can tune into specific channels to receive the news, without directly interacting with the news station. More on [ROS 2 Topics](https://docs.ros.org/en/humble/Concepts/Basic/Concepts-Turtlesim.html).

### Services: Synchronous Request-Response

**Services** are designed for **synchronous, one-to-one, request-response communication**. They are ideal for tasks that require an immediate response to a specific query.

-   **Synchronous**: When a client sends a request to a service server, the client typically blocks its execution until it receives a response from the server.
-   **One-to-one**: A service client invokes a specific service offered by a single service server.
-   **Request/Response**: Services utilize a defined request message and a corresponding response message (defined in `.srv` files).

**Use Cases**: Querying a robot's current status, triggering a specific action that provides a direct result (e.g., "calibrate sensor," "get joint state"), or calculating a value based on input. Dive deeper into [ROS 2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).

### Actions: Long-Running Goals with Feedback

**Actions** build upon the request-response model of services but are specifically tailored for **long-running, preemptible tasks that require continuous feedback**. They are implemented using a client-server architecture, similar to services.

-   **Asynchronous Goal**: An action client sends a goal to an action server and does not block while the goal is being processed.
-   **Feedback**: The action server can periodically send feedback to the client about the progress of the task.
-   **Preemptible**: The action client can send a cancel request to the server at any time, allowing the server to abort the current goal.
-   **Result**: Once the goal is completed (or canceled), the action server sends a final result back to the client.

**Use Cases**: High-level navigation goals (e.g., "Go to the kitchen"), complex manipulation tasks (e.g., "Pick up an object"), or any task that takes a significant amount of time and for which progress updates are useful. More details on [ROS 2 Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).

### Comparison Table: Communication Mechanisms

| Feature       | Topics                                  | Services                                  | Actions                                          |
| :------------ | :-------------------------------------- | :---------------------------------------- | :----------------------------------------------- |
| **Paradigm**  | Publish-Subscribe                       | Request-Response                          | Goal-Feedback-Result                             |
| **Sync**      | Asynchronous                            | Synchronous (Client blocks)               | Asynchronous (Client doesn't block)              |
| **Cardinality**| One-to-many                             | One-to-one                                | One-to-one                                       |
| **Primary Use**| Continuous, streaming data (e.g., sensor data, odometry) | Instantaneous queries, single command with immediate result | Long-duration tasks with progress monitoring and cancelation |
| **Example**   | Camera images, LiDAR scans              | Get current robot pose, toggle a light    | Navigate to a waypoint, pick up an object      |

## Quality of Service (QoS) Profiles

ROS 2 introduced **Quality of Service (QoS)** profiles to provide fine-grained control over the communication behavior between nodes. QoS settings allow you to configure parameters like reliability, durability, and history for each publisher and subscriber. This is crucial for adapting communication to different types of data and network conditions. You can read more about [ROS 2 QoS Policies here](https://docs.ros.org/en/humble/Concepts/Basic/About-Quality-of-Service-Settings.html).

-   **Reliability**: Guarantees delivery of messages (important for commands) or prefers speed (for sensor data).
-   **Durability**: Whether late-joining subscribers receive previously published messages.
-   **History**: How many messages to keep in a buffer.
-   **Liveliness**: How ROS 2 detects if a publisher or subscriber is still active.

## Your First ROS 2 System (Revisited)

Let's revisit our "hello world" system and make it more robust by incorporating QoS settings.

### The Talker Node (Python) with QoS

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        
        # Define a QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # New subscribers get the last message
        )
        
        self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
        self.get_logger().info('Talker node started, publishing to /chatter with RELIABLE QoS.')
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### The Listener Node (Python) with QoS

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        
        # Define a QoS profile for reliable communication (must match publisher)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos_profile # Use the defined QoS profile
        )
        self.get_logger().info('Listener node started, subscribing to /chatter with RELIABLE QoS.')
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Insights and Best Practices: Navigating the ROS 2 Ecosystem

-   **QoS Consistency**: Always ensure that QoS profiles between publishers and subscribers (or action/service clients and servers) are compatible. Mismatched QoS can lead to communication failures.
-   **Lifecycle Management**: For production systems, leverage ROS 2's managed lifecycle nodes. These allow for controlled transitions between states (unconfigured, inactive, active) for better system robustness. [Learn about ROS 2 Managed Nodes](https://docs.ros.org/en/humble/Concepts/About-Managed-Nodes.html).
-   **Logging**: Use `self.get_logger().info()`, `warn()`, `error()`, etc., for effective debugging and monitoring. Avoid `print()` statements in ROS 2 nodes.
-   **Launch Files**: For any system with more than one node, use ROS 2 launch files (`.py` or `.xml`). They provide a declarative way to start, configure, and manage multiple nodes and processes.
    ```python
    # Example of a simple launch file (my_robot_launch.py)
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='my_robot_pkg',
                executable='talker_node',
                name='my_talker',
                output='screen',
            ),
            Node(
                package='my_robot_pkg',
                executable='listener_node',
                name='my_listener',
                output='screen',
            ),
        ])
    ```
    (To run: `ros2 launch my_robot_pkg my_robot_launch.py`). More on [ROS 2 Launch Files](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launch-Files/Launch-Files.html).

## Summary

In this chapter, we have deepened our understanding of ROS 2 fundamentals. We dissected the roles of nodes, topics, services, and actions, providing a clear comparison of their use cases. We also explored the critical concept of Quality of Service (QoS) and its importance in tailoring communication for different robotic needs. Through enhanced Python examples and practical insights, you are now better equipped to design and implement robust, distributed robotic systems using ROS 2.

## Self-Assessment Questions

<details>
  <summary>1. What is a ROS 2 node and why is it important to keep them small and focused?</summary>
  <div>
    A ROS 2 node is a fundamental unit of computation, responsible for a single, well-defined task. Keeping them small and focused improves modularity, making the system easier to debug, test, and reuse.
  </div>
</details>

<details>
  <summary>2. What is the difference between a ROS 2 topic, a service, and an action? Give a use case for each.</summary>
  <div>
    - **Topic**: Asynchronous, one-to-many communication for continuous data streams (e.g., publishing camera images from a `camera_driver` node).
    - **Service**: Synchronous, one-to-one communication for quick, transactional tasks (e.g., a `localization` node offering a service to `get_robot_pose`).
    - **Action**: Asynchronous, one-to-one communication for long-running, preemptible tasks with feedback (e.g., a `navigation` node taking an action goal to `navigate_to_waypoint`).
  </div>
</details>

<details>
  <summary>3. Explain the concept of Quality of Service (QoS) profiles in ROS 2 and provide an example where a specific QoS setting would be critical.</summary>
  <div>
    QoS profiles in ROS 2 allow developers to define communication behavior for publishers and subscribers, controlling aspects like reliability, history, and durability. For instance, for critical commands (e.g., E-stop), `ReliabilityPolicy.RELIABLE` is critical to ensure every message is delivered. For high-frequency sensor data (e.g., LiDAR scans), `ReliabilityPolicy.BEST_EFFORT` might be preferred to prioritize speed over guaranteed delivery, preventing backlogs.
  </div>
</details>

<details>
  <summary>4. What is the purpose of a launch file in ROS 2 and when should you use one?</summary>
  <div>
    A launch file in ROS 2 is a Python or XML script used to start, configure, and manage multiple ROS 2 nodes and other processes in a declarative way. You should use one for any system with more than a single node, as it automates setup, parameter assignment, and inter-node coordination, making complex systems easier to run and reproduce.
  </div>
</details>

<details>
  <summary>5. Describe how `rclpy.spin()` and `rclpy.spin_once()` differ in their execution of node callbacks.</summary>
  <div>
    - `rclpy.spin(node)`: This function blocks indefinitely, continuously processing all pending callbacks (timers, subscriptions, service requests, action goals) for the given node until the node is explicitly shut down or `rclpy.ok()` becomes `False`.
    - `rclpy.spin_once(node)`: This function processes only one pending callback (if any) and then returns immediately. It is useful in situations where you need to integrate ROS 2 event processing with other non-ROS tasks in a single-threaded loop, giving you more granular control over execution.
  </div>
</details>

## Next Steps

Now that you have a comprehensive understanding of the fundamentals of ROS 2, we will move on to the next module. We will learn how to create a digital twin of our robot and simulate it in a virtual environment. This is a crucial step in the robotics development process, as it allows us to test and iterate on our designs before deploying them to physical hardware, significantly accelerating the development cycle.