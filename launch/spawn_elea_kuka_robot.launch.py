#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    elea_amr_pkg = get_package_share_directory('elea_amr')
    pkg_dir = get_package_share_directory('elea_amr')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file to use'
    )
    
    # KUKA arm configuration arguments
    arm_prefix_arg = DeclareLaunchArgument(
        'arm_prefix',
        default_value='arm_',
        description='Prefix for KUKA arm components'
    )
    
    arm_x_arg = DeclareLaunchArgument(
        'arm_x',
        default_value='0.3',
        description='X position of the arm on the robot'
    )
    
    arm_y_arg = DeclareLaunchArgument(
        'arm_y',
        default_value='0.0',
        description='Y position of the arm on the robot'
    )
    
    arm_z_arg = DeclareLaunchArgument(
        'arm_z',
        default_value='0.35',
        description='Z position of the arm on the robot'
    )
    
    arm_roll_arg = DeclareLaunchArgument(
        'arm_roll',
        default_value='0.0',
        description='Roll orientation of the arm'
    )
    
    arm_pitch_arg = DeclareLaunchArgument(
        'arm_pitch',
        default_value='0.0',
        description='Pitch orientation of the arm'
    )
    
    arm_yaw_arg = DeclareLaunchArgument(
        'arm_yaw',
        default_value='0.0',
        description='Yaw orientation of the arm'
    )
    
    # Add use_digilab argument for world-specific LIDAR topics
    use_digilab_arg = DeclareLaunchArgument(
        'use_digilab',
        default_value='false',
        description='Use digilab world topics instead of empty world topics'
    )

    # Robot description
    robot_description = Command([
        'xacro ', os.path.join(elea_amr_pkg, 'urdf', 'elea_kuka_robot.urdf.xacro'),
        ' arm_prefix:=', LaunchConfiguration('arm_prefix'),
        ' arm_x:=', LaunchConfiguration('arm_x'),
        ' arm_y:=', LaunchConfiguration('arm_y'),
        ' arm_z:=', LaunchConfiguration('arm_z'),
        ' arm_roll:=', LaunchConfiguration('arm_roll'),
        ' arm_pitch:=', LaunchConfiguration('arm_pitch'),
        ' arm_yaw:=', LaunchConfiguration('arm_yaw')
    ])

    # Start robot state publisher
    urdf_file = os.path.join(pkg_dir, 'urdf', 'elea_kuka_robot.urdf.xacro')
    
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Robot state publisher node (original xacro-based)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_xacro',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-name', 'elea_kuka_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # ROS-Gazebo bridge (for empty world)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/world/empty/model/elea_kuka_robot/link/base_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '--ros-args', '-r', '/world/empty/model/elea_kuka_robot/link/base_link/sensor/lidar/scan:=/scan'
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_digilab'))
    )

    # Bridge for digilab world
    ros_gz_bridge_digilab = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge_digilab',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/world/digilab/model/elea_kuka_robot/link/base_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '--ros-args', '-r', '/world/digilab/model/elea_kuka_robot/link/base_link/sensor/lidar/scan:=/scan'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_digilab'))
    )

    # Static transform for lidar frame compatibility
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'elea_kuka_robot/base_link/lidar'],
        output='screen'
    )

    # Static transform for base_footprint
    base_footprint_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )

    # Laser scan frame filter to fix frame_id
    laser_frame_filter_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_filter',
        arguments=['0', '0', '0.35', '0', '0', '0', 'base_link', 'lidar_link'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        arm_prefix_arg,
        arm_x_arg,
        arm_y_arg,
        arm_z_arg,
        arm_roll_arg,
        arm_pitch_arg,
        arm_yaw_arg,
        use_digilab_arg,
        robot_state_publisher_node,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        ros_gz_bridge_digilab,
        static_transform_node,
        base_footprint_transform_node,
        laser_frame_filter_node
    ])