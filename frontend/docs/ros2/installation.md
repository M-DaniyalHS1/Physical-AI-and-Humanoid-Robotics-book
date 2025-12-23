---
sidebar_position: 2
title: "ROS 2 Installation"
---

# ROS 2 Installation

This guide will walk you through the installation process for ROS 2, focusing on configurations relevant to Physical AI and Humanoid Robotics applications.

## System Requirements

- **Operating System**: Ubuntu 22.04 (Jammy) LTS, Windows 10/11, or macOS
- **RAM**: Minimum 8GB, recommended 16GB or more
- **Disk Space**: 10GB+ for basic installation
- **Processor**: Multi-core processor recommended

## Ubuntu Installation

### 1. Set up Sources

```bash
# Add the ROS 2 GPG key and repository
sudo apt update && sudo apt install -y curl gnupg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2. Install ROS 2

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

### 3. Environment Setup

```bash
# Source the ROS 2 setup script (add to ~/.bashrc to make permanent)
source /opt/ros/humble/setup.bash
```

## Windows Installation

1. Install Visual Studio 2019 or 2022 with C++ development tools
2. Install Python 3.8 or newer
3. Install Open Robotics dependencies
4. Download and run the ROS 2 installer for Windows

## Verification

After installation, verify everything works:

```bash
# Run the talker/listener demo
source /opt/ros/humble/setup.bash  # On Ubuntu
ros2 run demo_nodes_cpp talker
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash  # On Ubuntu
ros2 run demo_nodes_cpp listener
```

You should see the talker and listener nodes communicating with each other.