#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('elea_amr')
    tb_gazebo_dir = get_package_share_directory('elea_tb_gazebo')
    
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.57025')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz', default='false')
    
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
        
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.57025',
        description='Specify z position of the robot')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Full path to world model file to load')
        
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 automatically with this launch file.')

    # Specify the actions
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': [world, ' -r']
        }.items()
    )

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

    # Bridge between ROS2 and Ignition Gazebo
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/elea_robot/link/base_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '--ros-args', '-r', '/world/empty/model/elea_robot/link/base_link/sensor/lidar/scan:=/scan'
            
        ],
        output='screen')

   
    # Static transform from lidar_link to match Gazebo frame naming
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'elea_robot/base_link/lidar'],
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
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(static_transform_node)
    ld.add_action(rviz_node)

    return ld
