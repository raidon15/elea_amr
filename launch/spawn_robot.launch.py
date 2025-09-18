#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('elea_amr')
    
    # Get package directories for digilab world setup
    try:
        pkg_turtlebot4_ignition_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
        pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory('turtlebot4_ignition_gui_plugins')
        pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
        pkg_irobot_create_description = get_package_share_directory('irobot_create_description')
        pkg_irobot_create_ignition_bringup = get_package_share_directory('irobot_create_ignition_bringup')
        pkg_irobot_create_ignition_plugins = get_package_share_directory('irobot_create_ignition_plugins')
        digilab_available = True
    except:
        digilab_available = False
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.6s')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz', default='false')
    use_digilab = LaunchConfiguration('use_digilab', default='false')
    
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
    
    declare_use_digilab_cmd = DeclareLaunchArgument(
        'use_digilab',
        default_value='false',
        description='Use digilab world instead of empty world')

    # Environment variables for digilab world
    resource_paths = []
    
    # Add digilab paths if they exist
    digilab_map_path = '/home/benjamin/ros2_ws/install/ik_rmf_maps/share/ik_rmf_maps/maps/digilab'
    if os.path.exists(digilab_map_path):
        resource_paths.extend([
            digilab_map_path,
            digilab_map_path + '/models'
        ])
    
    # Add ik_rmf_assets if available
    try:
        ik_rmf_assets_models = os.path.join(get_package_share_directory('ik_rmf_assets'), 'models')
        resource_paths.append(ik_rmf_assets_models)
    except:
        pass
    
    # Add standard gazebo models path
    resource_paths.append(os.path.expandvars('$HOME/.gazebo/models'))
    
    # Add turtlebot4 paths if available
    if digilab_available:
        resource_paths.extend([
            os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'),
            os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'),
            str(Path(pkg_turtlebot4_description).parent.resolve()),
            str(Path(pkg_irobot_create_description).parent.resolve())
        ])
    
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=':'.join(resource_paths)
    )

    gui_plugin_paths = []
    if digilab_available:
        gui_plugin_paths.extend([
            os.path.join(pkg_turtlebot4_ignition_gui_plugins, 'lib'),
            os.path.join(pkg_irobot_create_ignition_plugins, 'lib')
        ])
    
    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=':'.join(gui_plugin_paths)
    )

    # Specify the actions - conditional Gazebo launch
    # For digilab world
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    start_digilab_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('gz_args', ['digilab.world -r -v 4'])
        ],
        condition=IfCondition(use_digilab)
    )

    # For regular world
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments={
            'gz_args': [world, ' -r']
        }.items(),
        condition=UnlessCondition(use_digilab)
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
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
        arguments=['0', '0', '0.255125', '0', '0', '0', 'base_link', 'lidar_link'],
        output='screen'
    )

    # RViz
    rviz_config_file = '/workspaces/elea_ws/src/elea_amr/rviz/elea_amr.rviz'
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

    # Add environment variables first
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_use_digilab_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_digilab_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(bridge_digilab_cmd)
    ld.add_action(static_transform_node)
    ld.add_action(base_footprint_transform_node)
    ld.add_action(laser_frame_filter_node)
    ld.add_action(rviz_node)

    return ld
