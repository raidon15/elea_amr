
<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [ELEA AMR Launch Guide](#elea-amr-launch-guide)
  - [📁 Available Launch Files](#-available-launch-files)
    - [1. `launch_gazebo.launch.py` - Gazebo World Only](#1-launch_gazebolaunchpy---gazebo-world-only)
    - [2. `spawn_robot_only.launch.py` - Robot Spawning Only](#2-spawn_robot_onlylaunchpy---robot-spawning-only)
    - [3. `complete_simulation.launch.py` - Full Simulation](#3-complete_simulationlaunchpy---full-simulation)
    - [4. `spawn_robot.launch.py` - Legacy Combined Launcher](#4-spawn_robotlaunchpy---legacy-combined-launcher)
  - [🚀 Recommended Workflows](#-recommended-workflows)
    - [Development Workflow (Separate Launches)](#development-workflow-separate-launches)
    - [Production Workflow (Combined Launch)](#production-workflow-combined-launch)
  - [🎮 Robot Control](#-robot-control)
    - [Basic Movement Commands](#basic-movement-commands)
  - [🔧 Troubleshooting](#-troubleshooting)
    - [Common Issues](#common-issues)
    - [Useful Debug Commands](#useful-debug-commands)
  - [📋 Robot Specifications](#-robot-specifications)
  - [🎯 Quick Reference](#-quick-reference)

<!-- /code_chunk_output -->

# ELEA AMR Launch Guide

This document describes all available launch files and usage options for the ELEA AMR robot simulation.

## 📁 Available Launch Files

### 1. `launch_gazebo.launch.py` - Gazebo World Only
**Purpose**: Launches Gazebo simulation with world selection

**Features**:
- Environment setup for digilab world resources
- Conditional world loading (empty vs digilab)
- Clock bridge for ROS-Gazebo time synchronization
- No robot spawning

**Usage**:
```bash
# Launch empty world (default)
ros2 launch elea_amr launch_gazebo.launch.py

# Launch digilab world
ros2 launch elea_amr launch_gazebo.launch.py use_digilab:=true

# Launch specific world file
ros2 launch elea_amr launch_gazebo.launch.py world:=my_world.sdf
```

**Parameters**:
- `world` (default: `empty.sdf`) - Path to world file
- `use_digilab` (default: `false`) - Use digilab world instead of empty world

---

### 2. `spawn_robot_only.launch.py` - Robot Spawning Only
**Purpose**: Spawns and configures the robot (requires Gazebo already running)

**Features**:
- Robot state publisher with URDF
- Entity spawning with position control
- ROS-Gazebo bridges (conditional for different worlds)
- Transform publishers (TF)
- Optional RViz visualization

**Usage**:
```bash
# Basic robot spawn (assumes Gazebo running)
ros2 launch elea_amr spawn_robot_only.launch.py

# Spawn at specific position
ros2 launch elea_amr spawn_robot_only.launch.py x_pose:=1.0 y_pose:=2.0 z_pose:=0.5

# Spawn with RViz
ros2 launch elea_amr spawn_robot_only.launch.py rviz:=true

# Spawn for digilab world
ros2 launch elea_amr spawn_robot_only.launch.py use_digilab:=true

# Full configuration
ros2 launch elea_amr spawn_robot_only.launch.py x_pose:=10.0 y_pose:=-30.0 rviz:=true use_digilab:=true
```

**Parameters**:
- `x_pose` (default: `0.0`) - Robot X position in meters
- `y_pose` (default: `0.0`) - Robot Y position in meters  
- `z_pose` (default: `0.57025`) - Robot Z position in meters
- `rviz` (default: `false`) - Launch RViz2 for visualization
- `use_digilab` (default: `false`) - Use digilab world topic mappings

---

### 3. `complete_simulation.launch.py` - Full Simulation
**Purpose**: Complete simulation launcher (Gazebo + Robot with timing)

**Features**:
- Launches Gazebo world
- Event-driven robot spawning after Gazebo initialization
- All robot configuration and bridges
- Integrated parameter forwarding

**Usage**:
```bash
# Complete simulation with empty world
ros2 launch elea_amr complete_simulation.launch.py

# Complete simulation with digilab world
ros2 launch elea_amr complete_simulation.launch.py use_digilab:=true

# Full configuration with positioning
ros2 launch elea_amr complete_simulation.launch.py use_digilab:=true x_pose:=10.0 y_pose:=-30.0 rviz:=true

# Custom world with positioning
ros2 launch elea_amr complete_simulation.launch.py world:=my_world.sdf x_pose:=5.0 y_pose:=5.0
```

**Parameters**:
- `world` (default: `empty.sdf`) - Path to world file
- `use_digilab` (default: `false`) - Use digilab world
- `x_pose` (default: `10.0`) - Robot X position
- `y_pose` (default: `-30.0`) - Robot Y position
- `z_pose` (default: `0.57025`) - Robot Z position
- `rviz` (default: `false`) - Launch RViz2

---

### 4. `spawn_robot.launch.py` - Legacy Combined Launcher
**Purpose**: Original combined launcher (maintained for compatibility)

**Usage**:
```bash
# Legacy combined launch
ros2 launch elea_amr spawn_robot.launch.py

# With digilab world
ros2 launch elea_amr spawn_robot.launch.py use_digilab:=true
```

---

## 🚀 Recommended Workflows

### Development Workflow (Separate Launches)
**Best for**: Development, testing, debugging

```bash
# Terminal 1: Start Gazebo first
ros2 launch elea_amr launch_gazebo.launch.py use_digilab:=true

# Terminal 2: Spawn robot when ready
ros2 launch elea_amr spawn_robot_only.launch.py use_digilab:=true x_pose:=10.0 y_pose:=-30.0 rviz:=true
```

**Benefits**:
- Independent control of Gazebo and robot
- Easy to restart robot without restarting world
- Better for debugging and development
- Can test world changes without robot interference

### Production Workflow (Combined Launch)
**Best for**: Demos, automated testing, production use

```bash
# Single command for complete setup
ros2 launch elea_amr complete_simulation.launch.py use_digilab:=true x_pose:=10.0 y_pose:=-30.0 rviz:=true
```

**Benefits**:
- Single command execution
- Automatic timing and coordination
- Consistent startup sequence
- Good for demonstrations

---

## 🎮 Robot Control

After robot is spawned, control it using velocity commands:

### Basic Movement Commands

**Move Forward**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10
```

**Stop Robot**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

**Turn Left (while moving)**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --rate 10
```

**Turn Right (while moving)**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}" --rate 10
```

**Rotate in Place**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" --rate 10
```

**Move Backward**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10
```

---

## 🔧 Troubleshooting

### Common Issues

**1. Robot not spawning**
- Ensure Gazebo is fully loaded before spawning robot
- Check if world is properly loaded
- Verify robot description path

**2. No movement response**
- Check if robot is properly spawned: `ros2 topic list | grep cmd_vel`
- Verify bridge is running: `ros2 node list | grep bridge`
- Check for error messages in terminal

**3. Digilab world not loading**
- Verify digilab packages are installed
- Check resource paths are correctly set
- Some digilab models might be missing (warnings are normal)

**4. RViz issues**
- Ensure robot_state_publisher is running
- Check TF transforms: `ros2 run tf2_tools view_frames`
- Verify URDF is loading correctly

### Useful Debug Commands

**Check active topics**:
```bash
ros2 topic list
```

**Check active nodes**:
```bash
ros2 node list
```

**Monitor robot position**:
```bash
ros2 topic echo /odom
```

**Check TF tree**:
```bash
ros2 run tf2_tools view_frames
```

**Monitor velocity commands**:
```bash
ros2 topic echo /cmd_vel
```

---

## 📋 Robot Specifications

- **Dimensions**: 1.08m (length) × 0.63m (width) × 0.7m (height)
- **Wheel separation**: 0.63m
- **Wheel radius**: 0.06m
- **Visual mesh**: KMP200.dae
- **Sensors**: LIDAR (360° scan, 30m range)
- **Drive type**: Differential drive

---

## 🎯 Quick Reference

| Task | Command |
|------|---------|
| Launch empty world | `ros2 launch elea_amr launch_gazebo.launch.py` |
| Launch digilab world | `ros2 launch elea_amr launch_gazebo.launch.py use_digilab:=true` |
| Spawn robot only | `ros2 launch elea_amr spawn_robot_only.launch.py` |
| Complete simulation | `ros2 launch elea_amr complete_simulation.launch.py use_digilab:=true` |
| Move robot forward | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" --rate 10` |
| Stop robot | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once` |

---

*Last updated: September 17, 2025*
