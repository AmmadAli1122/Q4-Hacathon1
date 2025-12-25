# Quickstart Guide: Physical AI — Module 3: The AI-Robot Brain (NVIDIA Isaac™)

## Overview
This guide provides the essential steps to get started with NVIDIA Isaac technologies for robotic perception and navigation as covered in Module 3.

## Prerequisites
- NVIDIA GPU with CUDA support (minimum: GTX 1060 or equivalent)
- ROS 2 Humble Hawksbill installed
- Isaac Sim installed and configured
- Isaac ROS packages installed

## Environment Setup

### 1. Verify Hardware Requirements
```bash
# Check for NVIDIA GPU
nvidia-smi

# Verify CUDA installation
nvcc --version
```

### 2. Install Isaac Sim
```bash
# Clone Isaac Sim repository
git clone https://github.com/NVIDIA-Omniverse/isaac-sim.git

# Follow installation instructions in the repository
# Ensure proper GPU drivers are installed
```

### 3. Install Isaac ROS Packages
```bash
# Add NVIDIA ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://repos.ros.org/repos.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-common
```

## Basic Isaac Sim Workflow

### 1. Launch Isaac Sim
```bash
# Navigate to Isaac Sim directory
cd isaac-sim

# Launch Isaac Sim
./isaac-sim.sh
```

### 2. Create Basic Simulation Environment
- Open Isaac Sim
- Create a new scene with basic objects
- Add a robot model (e.g., simple wheeled robot to start)
- Configure sensors (camera, LiDAR, IMU)

### 3. Export Robot for ROS 2 Integration
- Configure the robot with ROS 2 bridges
- Export robot configuration as URDF
- Verify ROS 2 topic connections

## Isaac ROS Perception Pipeline

### 1. Basic Perception Setup
```bash
# Launch Isaac ROS perception pipeline
ros2 launch isaac_ros_apriltag_interfaces isaac_ros_apriltag.launch.py
```

### 2. GPU Accelerated Processing
- Verify GPU acceleration is working
- Monitor GPU usage during perception tasks
- Check that Isaac ROS packages are using CUDA

## Navigation with Nav2

### 1. Setup Navigation Stack
```bash
# Install Nav2 packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Launch basic navigation
ros2 launch nav2_bringup navigation_launch.py
```

### 2. Configure for Humanoid Robots
- Adjust navigation parameters for bipedal movement
- Configure stability constraints
- Set appropriate path planning algorithms

## Chapter-Specific Workflows

### Chapter 1: NVIDIA Isaac Ecosystem
- Explore Isaac Sim interface and basic simulation
- Understand the relationship between Isaac tools and ROS 2
- Set up basic robot simulation environment

### Chapter 2: Perception with Isaac Sim & Isaac ROS
- Create synthetic data generation pipeline
- Implement domain randomization techniques
- Integrate perception outputs with ROS 2

### Chapter 3: Navigation & Nav2
- Configure Nav2 for humanoid robot model
- Implement bipedal path planning
- Test navigation in simulated environments

## Troubleshooting

### Common Issues
- GPU not detected: Verify NVIDIA drivers and CUDA installation
- Isaac Sim crashes: Check GPU memory and system requirements
- ROS 2 connection issues: Verify network configuration and topic names
- Perception performance: Ensure Isaac ROS packages are using GPU acceleration

## Next Steps
After completing this quickstart, proceed to Chapter 1 to dive deeper into the NVIDIA Isaac ecosystem and its role in Physical AI.