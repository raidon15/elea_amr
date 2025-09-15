# ELEA AMR Robot Package

A ROS 2 package for the ELEA Autonomous Mobile Robot (AMR) with differential drive configuration.

## Robot Specifications

- **Length**: 1.849 m
- **Width**: 1.445 m  
- **Height**: 1.0205 m
- **Drive System**: Differential drive with 2 motorized wheels
- **Support**: 2 caster wheels (front and rear)
- **Wheel Radius**: 0.06 m
- **Wheel Separation**: 1.485 m

## Package Contents

- `urdf/elea_diff_robot.urdf` - Robot description file
- `launch/spawn_robot.launch.py` - Launch file for Ignition Gazebo simulation

## Features

- Differential drive control via `/cmd_vel` topic
- Odometry feedback via `/odom` topic
- Ignition Gazebo simulation support
- ROS 2 Bridge for Ignition-ROS 2 communication

## Dependencies

- ROS 2 Humble
- Ignition Gazebo 6
- robot_state_publisher
- ros_gz_sim
- ros_gz_bridge

## Building

```bash
cd <your_workspace>
colcon build --packages-select elea_amr
source install/setup.bash
```

## Usage

### Launch Robot in Simulation

```bash
ros2 launch elea_amr spawn_robot.launch.py
```

### Control the Robot

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Monitor Odometry

```bash
ros2 topic echo /odom
```

## License

TODO: Add license information

## Author

Benjamin
