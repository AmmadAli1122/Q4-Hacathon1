---
title: Chapter 3 - From Code to Body
sidebar_label: "Chapter 3: From Code to Body"
---

# Chapter 3: From Code to Body — Python Agents, rclpy, and URDF

## Introduction

In the previous chapters, we explored ROS 2 as the robotic nervous system and learned about its communication patterns. Now, we'll bridge the gap between Python AI agents and physical robot components using ROS 2's Python client library (rclpy) and the Unified Robot Description Format (URDF). This chapter will show you how to connect your Python code to the physical world of robotics.

## Bridging Python AI Agents to ROS 2 with rclpy

Python is one of the most popular languages for AI development, with rich libraries for machine learning, computer vision, and data processing. The rclpy library allows Python programs to integrate seamlessly with ROS 2 systems, enabling AI agents to interact with robot hardware.

### What is rclpy?

rclpy is the Python client library for ROS 2. It provides Python bindings for ROS 2 concepts like nodes, publishers, subscribers, services, and actions. With rclpy, you can create ROS 2 nodes in Python that communicate with nodes written in other languages like C++.

### Installing rclpy

rclpy is typically installed as part of the ROS 2 Python development packages. You can install it in your Python environment:

```bash
pip install rclpy
```

### Creating Your First Python Node

Let's create a simple Python node that acts as an AI agent:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Create a subscriber to receive sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )

        # Create a publisher to send commands
        self.publisher = self.create_publisher(
            String,
            'ai_commands',
            10
        )

        self.get_logger().info('AI Agent Node initialized')

    def laser_callback(self, msg):
        # Process sensor data with AI logic
        ranges = np.array(msg.ranges)
        # Remove invalid readings
        valid_ranges = ranges[np.isfinite(ranges)]

        # Simple AI: if something is close in front, send a stop command
        if len(valid_ranges) > 0 and np.min(valid_ranges) < 1.0:  # Less than 1 meter
            cmd_msg = String()
            cmd_msg.data = 'STOP'
            self.publisher.publish(cmd_msg)
            self.get_logger().info('Obstacle detected, STOP command sent')

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced AI Integration Patterns

#### Pattern 1: Perception Pipeline
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Apply AI perception algorithms
        processed_image = self.detect_objects(cv_image)

        # Publish results
        # (Additional code to publish processed results)

    def detect_objects(self, image):
        # Placeholder for AI object detection
        # In practice, you'd use models like YOLO, SSD, etc.
        return image
```

#### Pattern 2: Behavior Trees
```python
import rclpy
from rclpy.node import Node
import py_trees

class BehaviorTreeAgent(Node):
    def __init__(self):
        super().__init__('behavior_tree_agent')

        # Create behavior tree
        self.root = self.create_behavior_tree()

        # Timer to tick the tree
        self.timer = self.create_timer(0.1, self.tick_tree)

    def create_behavior_tree(self):
        # Create a simple patrol behavior
        root = py_trees.composites.Sequence('Patrol')
        check_battery = py_trees.behaviours.CheckBattery('CheckBattery')
        navigate = py_trees.behaviours.Navigate('Navigate')

        root.add_children([check_battery, navigate])
        return root

    def tick_tree(self):
        self.root.tick_once()
```

## Understanding URDF: Links, Joints, and Frames

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical structure of a robot including links, joints, and their relationships.

### Links: The Building Blocks

A link represents a rigid body in the robot. Each link has:
- Physical properties (mass, inertia, visual appearance)
- Collision properties
- A coordinate frame

Example of a simple link:
```xml
<link name="base_link">
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0"/>
    <geometry>
      <box size="0.5 0.5 0.2"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0"/>
    <geometry>
      <box size="0.5 0.5 0.2"/>
    </geometry>
  </collision>
</link>
```

### Joints: Connecting Links

Joints define the kinematic and dynamic relationships between links. Types of joints include:
- Fixed: No movement
- Revolute: Rotational movement around an axis
- Continuous: Rotational movement without limits
- Prismatic: Linear movement along an axis
- Floating: 6 degrees of freedom
- Planar: Movement in a plane

Example of a joint:
```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.2 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

### Frames: Coordinate Systems

Each link has its own coordinate frame. TF (Transform) trees allow ROS to understand the spatial relationships between all frames in the robot and environment.

### Modeling Humanoid Robots

Humanoid robots require special considerations in URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Additional links and joints for arms, legs, etc. -->
</robot>
```

## How URDF Enables Simulation and Real-World Deployment

### Simulation Benefits
- Physics simulation with accurate dynamics
- Sensor simulation (camera, LIDAR, IMU)
- Collision detection
- Visualization in tools like RViz and Gazebo

### Real-World Deployment
- Robot state publishing (robot_state_publisher)
- Forward and inverse kinematics
- Motion planning compatibility
- Hardware interface mapping

## Preparing Robot Description for Later Simulation Modules

When creating URDF for your robot, consider these elements that will be important for later simulation modules:

### 1. Proper Inertial Properties
```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
</inertial>
```

### 2. Collision and Visual Elements
```xml
<visual>
  <origin xyz="0 0 0"/>
  <geometry>
    <box size="0.5 0.5 0.2"/>
  </geometry>
</visual>
<collision>
  <origin xyz="0 0 0"/>
  <geometry>
    <box size="0.5 0.5 0.2"/>
  </geometry>
</collision>
```

### 3. Transmission Elements for Actuators
```xml
<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Integration Example: Python AI Agent with URDF Robot

Here's a complete example that combines Python AI with a URDF robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publishers for robot commands
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )

        # Subscribers for sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.joint_positions = {}

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def control_loop(self):
        # Simple AI controller logic
        cmd_msg = JointState()
        cmd_msg.name = ['left_shoulder_joint', 'right_shoulder_joint']
        cmd_msg.position = [0.1, -0.1]  # Move arms slightly

        self.joint_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter has shown you how to bridge Python AI agents with physical robot components using rclpy and how to describe robots using URDF. You've learned about:

- The rclpy library for Python-ROS 2 integration
- How to create AI agents as ROS 2 nodes
- The structure of URDF with links, joints, and frames
- How to model humanoid robots in URDF
- The role of URDF in simulation and real-world deployment

These concepts form the foundation for connecting your AI algorithms to real robotic systems, preparing you for the simulation, perception, and autonomy modules that will follow.

## Exercises

1. Create a simple URDF for a 2-wheeled robot with proper links, joints, and inertial properties.

2. Write a Python node that subscribes to a camera topic and publishes a command to move the robot forward if no obstacles are detected in the image.

3. Design a URDF for a simple robotic arm with at least 3 joints and implement a Python controller that moves the arm to a specific position.