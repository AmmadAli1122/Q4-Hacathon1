---
title: Chapter 2 - Communication in Motion
sidebar_label: "Chapter 2: Communication in Motion"
---

# Chapter 2: Communication in Motion — Nodes, Topics, and Services

## Introduction

In the previous chapter, we explored ROS 2 as the robotic nervous system and learned about its distributed architecture. Now, we'll dive deeper into the specific communication mechanisms that make this distributed system work: nodes, topics, and services. Understanding these communication patterns is essential for designing effective robotic systems.

## ROS 2 Nodes: Autonomous Computational Units

Nodes are the fundamental building blocks of any ROS 2 system. Each node is an autonomous computational unit that performs a specific task within the robotic system. A node can be thought of as a process that implements part of the robot's functionality.

### Creating a Node

In Python, nodes are created by extending the `Node` class from `rclpy`. Here's a basic example:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    rclpy.spin(minimal_node)

    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Nodes are autonomous, meaning they can run independently and make their own decisions. They communicate with other nodes through topics, services, and other mechanisms we'll explore.

## Topics: Continuous Data Streams

Topics are the primary mechanism for continuous data streams in ROS 2. They use a publish-subscribe pattern where nodes can publish data to a topic and other nodes can subscribe to that topic to receive the data.

### How Topics Work

1. **Publisher**: A node that sends data to a topic
2. **Subscriber**: A node that receives data from a topic
3. **Topic**: The named channel through which data flows

Topics are ideal for:
- Sensor data (camera images, LIDAR scans, IMU readings)
- Robot state information (position, velocity)
- Continuous control commands
- System status updates

### Example: Publisher and Subscriber

Here's a simple example of a publisher node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And here's a corresponding subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Request-Response Interactions

While topics are excellent for continuous data streams, sometimes you need request-response interactions. This is where services come in. Services allow a node to send a request and receive a response, similar to a function call across the network.

### How Services Work

1. **Service Client**: A node that sends a request to a service
2. **Service Server**: A node that receives the request, processes it, and sends back a response
3. **Service**: The named endpoint that handles the request-response interaction

Services are ideal for:
- Actions that need confirmation (moving to a specific position)
- Querying robot state (current battery level)
- Triggering specific behaviors (starting navigation)
- Configuration changes

### Example: Service Server and Client

First, let's define a service request. You would typically create a `.srv` file, but here's a conceptual example:

```python
# Service server example
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        # Define service here (would use custom service type in practice)
        # self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Design Patterns for Scalable Robot Communication

### Publisher-Subscriber Pattern
- Decouples publishers from subscribers
- Enables one-to-many communication
- Provides loose coupling between nodes
- Supports different update frequencies

### Client-Server Pattern (Services)
- Provides synchronous request-response
- Ensures request processing
- Good for actions requiring confirmation
- Enables remote procedure calls

### Action Pattern
- For long-running tasks with feedback
- Provides goal, feedback, and result
- Allows for preemption and cancellation
- Suitable for navigation, manipulation tasks

### Parameter Server Pattern
- Centralized configuration management
- Dynamic parameter updates
- Type-safe parameter access
- Node-specific and global parameters

## When to Use Topics vs Services

### Use Topics When:
- You need continuous data streams
- Multiple subscribers need the same data
- You don't need confirmation of receipt
- Data is time-sensitive (sensors, state)
- Communication is asynchronous

### Use Services When:
- You need request-response interaction
- You require confirmation of action
- The operation has a clear start and end
- You need to query information
- Synchronous communication is acceptable

## Practical Considerations

### Quality of Service (QoS) Settings
ROS 2 provides QoS settings that allow you to tune communication behavior:
- Reliability (reliable vs. best-effort)
- Durability (transient-local vs. volatile)
- History (keep-all vs. keep-last)
- Rate limiting and buffering

### Message Types
ROS 2 comes with standard message types:
- `std_msgs`: Basic data types (String, Int32, Float64, etc.)
- `sensor_msgs`: Sensor data (Image, LaserScan, Imu, etc.)
- `geometry_msgs`: Spatial information (Point, Pose, Twist, etc.)
- `nav_msgs`: Navigation-related messages
- Custom message types for specific applications

## Summary

Communication patterns in ROS 2 are fundamental to creating effective robotic systems. Topics provide continuous data streams ideal for sensor data and state information, while services offer request-response interactions for actions requiring confirmation. Understanding when to use each pattern is crucial for designing scalable and maintainable robotic systems.

The design patterns we've covered provide a foundation for building complex robotic applications with proper separation of concerns and appropriate communication mechanisms.

## Exercises

1. Design a communication pattern for a robot with a camera, where you want to save images when a specific object is detected. Would you use topics, services, or both? Explain your reasoning.

2. Create a simple publisher that publishes the current time every second, and a subscriber that logs the time difference between consecutive messages.

3. Identify three scenarios in a mobile robot application where you would use services instead of topics, and explain why.