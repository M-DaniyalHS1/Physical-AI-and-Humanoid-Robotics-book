---
sidebar_position: 3
title: "Simulation Integration with ROS 2"
---

# Simulation Integration with ROS 2

This section covers how to integrate simulation environments with ROS 2 for Physical AI and Humanoid Robotics applications.

## Gazebo-ROS 2 Integration

Gazebo provides native support for ROS 2 through the Gazebo ROS packages. This allows seamless communication between the simulation and your ROS 2 nodes.

### Gazebo Plugins for ROS 2

Gazebo uses plugins to interface with ROS 2. Common plugins include:

- `libgazebo_ros_diff_drive.so`: Differential drive controller
- `libgazebo_ros_joint_state_publisher.so`: Joint state publisher
- `libgazebo_ros_camera.so`: Camera sensor
- `libgazebo_ros_laser.so`: Laser scanner

Example robot URDF with Gazebo plugins:

```xml
<robot name="humanoid_robot">
  <!-- ... joint and link definitions ... -->
  
  <gazebo>
    <plugin filename="libgazebo_ros_joint_state_publisher.so" name="joint_state_publisher">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=joint_states</remapping>
      </ros>
      <update_rate>30</update_rate>
      <joint_name>hip_joint</joint_name>
      <joint_name>knee_joint</joint_name>
      <!-- Add other joint names -->
    </plugin>
  </gazebo>
</robot>
```

### Launching Complete Simulation

Create a launch file to start both Gazebo and your robot controllers:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            )
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'humanoid_robot'],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': True}]
        )
    ])
```

## Unity-ROS 2 Integration

Unity integration with ROS 2 requires additional setup using the Unity ROS TCP Connector.

### Setting up Unity for ROS 2 Communication

1. Create a ROS TCP Connector in Unity scene
2. Configure connection settings to match your ROS 2 setup
3. Create publishers and subscribers for robot data

### Unity Robot Control Example

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class UnityHumanoidController : MonoBehaviour
{
    private ROSConnection ros;
    
    [SerializeField] private string rosIP = "127.0.0.1";
    [SerializeField] private int rosPort = 10000;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIP, rosPort);
        
        // Subscribe to ROS topics
        ros.Subscribe<JointStateMsg>("/humanoid_robot/joint_commands", OnJointCommand);
    }
    
    void OnJointCommand(JointStateMsg cmd)
    {
        // Apply joint commands to Unity humanoid model
        for (int i = 0; i < cmd.name.Count; i++)
        {
            string jointName = cmd.name[i];
            float position = (float)cmd.position[i];
            // Set joint position in Unity
            SetJointPosition(jointName, position);
        }
    }
    
    void SetJointPosition(string jointName, float position)
    {
        // Implementation to set Unity joint position
    }
    
    void FixedUpdate()
    {
        // Publish robot state back to ROS
        var jointState = new JointStateMsg();
        // ... populate with current joint states
        ros.Publish("/humanoid_robot/joint_states", jointState);
    }
}
```

## Physical AI Simulation Workflows

### Training Pipeline

1. **Environment Setup**: Create diverse simulation environments
2. **Scenario Definition**: Define tasks and reward functions
3. **Data Collection**: Run simulations and collect state-action pairs
4. **Model Training**: Train AI models using collected data
5. **Validation**: Test trained models in simulation
6. **Transfer**: Deploy to real robots with domain adaptation

### Simulation-to-Reality Transfer

To bridge the sim-to-real gap:

- **Domain Randomization**: Randomize simulation parameters
- **System Identification**: Match simulation parameters to real robot
- **Adaptive Control**: Adjust for model discrepancies
- **Progressive Transfer**: Gradually increase task complexity

## Tools and Utilities

### RViz2 Integration
Visualize simulation data alongside real robot data in RViz2.

### Performance Monitoring
Monitor simulation performance to ensure real-time requirements.

### Debugging
Use ROS 2 tools to debug simulation-robot communication issues.