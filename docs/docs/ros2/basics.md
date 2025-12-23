---
sidebar_position: 3
title: "ROS 2 Basics"
---

# ROS 2 Basics

In this section, we'll cover fundamental ROS 2 concepts that are essential for Physical AI and Humanoid Robotics applications.

## Nodes

A **node** is a process that performs computation. ROS 2 is designed as a distributed system where multiple nodes communicate with each other. Nodes are typically organized by functionality:

```python
import rclpy
from rclpy.node import Node

class PhysicalAINode(Node):
    def __init__(self):
        super().__init__('physical_ai_node')
        self.get_logger().info('Physical AI Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Messages

**Topics** are named buses over which nodes exchange messages. This is the primary method of communication in ROS 2.

```python
# Publisher example
publisher = self.create_publisher(String, 'robot_status', 10)
msg = String()
msg.data = 'Robot ready for Physical AI tasks'
self.publisher.publish(msg)
```

## Services

**Services** provide a request/reply communication pattern. Useful for actions that require a response.

## Actions

**Actions** are a more advanced communication pattern for long-running tasks that provide feedback, goals, and results.

## Parameter System

Parameters allow configuration of nodes at runtime.

## Launch Files

Launch files allow you to start multiple nodes at once:

```xml
<launch>
  <node pkg="my_robot_package" exec="perception_node" name="perception"/>
  <node pkg="my_robot_package" exec="reasoning_node" name="reasoning"/>
  <node pkg="my_robot_package" exec="action_node" name="action"/>
</launch>
```

## Tools for Physical AI Development

### rqt
Graphical tool suite for visualizing ROS 2 data.

### rviz2
3D visualization tool for robot data, particularly important for Physical AI applications.

### ros2 topic, service, action commands
Command-line tools for introspection and debugging.