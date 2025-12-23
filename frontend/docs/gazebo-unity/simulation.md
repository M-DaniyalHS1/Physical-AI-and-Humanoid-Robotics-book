---
sidebar_position: 2
title: "Simulation Setup and Configuration"
---

# Simulation Setup and Configuration

This guide provides instructions for setting up simulation environments for Physical AI and Humanoid Robotics applications using both Gazebo and Unity.

## Gazebo Setup

### Installation

```bash
# Ubuntu
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control

# Install Gazebo Garden (latest version)
sudo apt install gazebo-garden
```

### Basic World Creation

Create a world file (`my_world.sdf`):

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physical_ai_world">
    <!-- Include environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add humanoid robot -->
    <include>
      <name>humanoid_robot</name>
      <uri>model://my_humanoid</uri>
    </include>
  </world>
</sdf>
```

### Launching Simulation

```bash
# Start Gazebo with a world
gazebo --verbose my_world.sdf

# Or launch with ROS 2
ros2 launch my_robot_bringup gazebo.launch.py
```

## Unity Setup for Robotics

### Installing Unity

1. Download Unity Hub from Unity's website
2. Install Unity 2022.3 LTS (Long Term Support)
3. Install required packages for robotics simulation

### Unity Robotics Simulation Package

Unity provides the Robotics Simulation package to integrate with ROS/ROS 2:

```bash
# In Unity Package Manager
com.unity.robotics.ros-tcp-connector
com.unity.robotics.urdf-importer
```

### Connecting Unity and ROS 2

The Unity Robotics package enables bidirectional communication between Unity and ROS 2:

```csharp
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>("joint_states");
    }

    void Update()
    {
        // Send robot state to ROS
        var jointState = new JointStateMsg();
        // ... populate message
        ros.Publish("joint_states", jointState);
    }
}
```

## Simulation for Physical AI Training

### Domain Randomization

To improve generalization of AI models trained in simulation:

- Vary lighting conditions
- Change material properties
- Add texture randomization
- Modify physics parameters within realistic ranges

### Sensor Simulation

Both Gazebo and Unity can simulate various sensors:

- **Cameras**: RGB, depth, stereo
- **LiDAR**: 2D and 3D scanners
- **IMU**: Inertial measurement units
- **Force/Torque sensors**: For contact detection
- **GPS**: Position estimation

## Best Practices

1. **Validation**: Always validate simulation results on real robots when possible
2. **Physics Accuracy**: Use appropriate physics parameters that match real-world values
3. **Sensor Noise**: Add realistic noise models to sensor data
4. **Timing**: Match real-time factors to ensure realistic behavior
5. **Environment Complexity**: Start with simple environments and increase complexity gradually