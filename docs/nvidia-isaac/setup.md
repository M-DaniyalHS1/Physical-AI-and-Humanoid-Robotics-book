---
sidebar_position: 2
title: "NVIDIA Isaac Setup"
---

# NVIDIA Isaac Setup

This guide will walk you through setting up NVIDIA Isaac for Physical AI and Humanoid Robotics applications.

## System Requirements

### Hardware Requirements
- **GPU**: NVIDIA GPU with compute capability 6.0 or higher (recommended: RTX 30/40 series or A40/A6000)
- **RAM**: 32GB minimum, 64GB or more recommended
- **Storage**: 50GB+ free space for Isaac Sim and dependencies
- **OS**: Ubuntu 20.04 LTS or 22.04 LTS (recommended for development)

### Software Requirements
- **CUDA**: 11.8 or later
- **Docker**: If using containerized deployment
- **ROS 2**: Humble Hawksbill (recommended)

## Installing NVIDIA Isaac Sim

### Prerequisites

1. Install NVIDIA GPU drivers:
```bash
sudo apt update
sudo ubuntu-drivers autoinstall
# Reboot after installation
```

2. Install CUDA (if not already installed):
```bash
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run
sudo sh cuda_12.3.0_545.23.06_linux.run
```

### Option 1: Isaac Sim via Omniverse Launcher (Recommended)

1. Download Omniverse Launcher from NVIDIA Developer website
2. Install Isaac Sim extension through the launcher
3. Launch Isaac Sim from the application menu

### Option 2: Isaac Sim via Docker

```bash
# Pull Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:4.0.0

# Run Isaac Sim
xhost +local:docker
docker run --gpus all -it --rm --network=host \
  --env "DISPLAY" \
  --env "QT_X11_NO_MITSHM=1" \
  --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume "/tmp/.docker.xauth:/tmp/.docker.xauth:rw" \
  --volume "/dev/shm:/dev/shm" \
  --volume "$(pwd):/workspace" \
  nvcr.io/nvidia/isaac-sim:4.0.0
```

## Isaac ROS Setup

Isaac ROS provides GPU-accelerated ROS 2 packages:

```bash
# Clone Isaac ROS repository
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Install dependencies
sudo apt update
sudo apt install ros-humble-isaac-ros-common

# Install specific packages (example: image pipeline)
sudo apt install ros-humble-isaac-ros-image-pipeline
```

## Isaac Lab Setup

For research and learning applications:

```bash
# Clone Isaac Lab repository
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# Create conda environment
conda env create -f source/extension_assets/omni.isaac.sim.python/config/isaacsim.env.yml
conda activate IsaacLab
```

## Verification

Test your installation with a simple example:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Test Isaac ROS packages
ros2 run isaac_ros_image_proc rectify
```

## Troubleshooting

### Common Issues

1. **GPU not detected**:
   - Ensure proper NVIDIA drivers and CUDA are installed
   - Check with `nvidia-smi`

2. **Isaac Sim fails to start**:
   - Verify X11 forwarding if using Docker
   - Check for OpenGL support

3. **Memory issues**:
   - Increase swap space if running on limited RAM systems
   - Close unnecessary applications

### Performance Optimization

- Set GPU to maximum performance mode: `sudo nvidia-smi -ac 5000,1590`
- Ensure sufficient RAM allocation for simulation
- Consider using SSD storage for faster asset loading