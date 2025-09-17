#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('elea_amr')
    
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='10.0')
    y_pose = LaunchConfiguration('y_pose', default='-30.0')
    z_pose = LaunchConfiguration('z_pose', default='0.57025')
    use_rviz = LaunchConfiguration('rviz', default='false')
    use_digilab = LaunchConfiguration('use_digilab', default='false')
    
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='10.0',
        description='Specify x position of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='-30.0',
        description='Specify y position of the robot')
        
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.57025',
        description='Specify z position of the robot')
        
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz2 automatically with this launch file.')
    
    declare_use_digilab_cmd = DeclareLaunchArgument(
        'use_digilab',
        default_value='false',
        description='Use digilab world topics instead of empty world topics')

    # Start robot state publisher
    urdf_file = os.path.join(pkg_dir, 'urdf', 'elea_diff_robot.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Spawn the robot
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'elea_robot',
                   '-x', x_pose,
                   '-y', y_pose,
                   '-z', z_pose],
        output='screen')

    # Bridge between ROS2 and Ignition Gazebo (for empty world)
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/world/empty/model/elea_robot/link/base_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '--ros-args', '-r', '/world/empty/model/elea_robot/link/base_link/sensor/lidar/scan:=/scan'
        ],
        output='screen',
        condition=UnlessCondition(use_digilab)
    )

    # Bridge for digilab world
    bridge_digilab_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/world/digilab/model/elea_robot/link/base_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '--ros-args', '-r', '/world/digilab/model/elea_robot/link/base_link/sensor/lidar/scan:=/scan'
        ],
        output='screen',
        condition=IfCondition(use_digilab)
    )

    # Static transform for lidar frame compatibility
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'elea_robot/base_link/lidar'],
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

    # RViz
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'elea_amr.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_use_digilab_cmd)

    # Add robot-related actions
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(bridge_digilab_cmd)
    ld.add_action(static_transform_node)
    ld.add_action(base_footprint_transform_node)
    ld.add_action(laser_frame_filter_node)
    ld.add_action(rviz_node)

    return ld
