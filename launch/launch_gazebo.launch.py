#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
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
    
    # Launch configuration variables
    world = LaunchConfiguration('world')
    use_digilab = LaunchConfiguration('use_digilab', default='false')
    
    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Full path to world model file to load')
    
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

    # Gazebo launch
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # For digilab world
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

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add environment variables first
    ld.add_action(ign_resource_path)
    ld.add_action(ign_gui_plugin_path)

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_digilab_cmd)

    # Add gazebo launch actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_digilab_gazebo_cmd)
    ld.add_action(clock_bridge)

    return ld
