#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('elea_amr')
    
    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('rviz')
    use_digilab = LaunchConfiguration('use_digilab')
    
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

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Full path to world model file to load')
        
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz2 automatically with this launch file.')
    
    declare_use_digilab_cmd = DeclareLaunchArgument(
        'use_digilab',
        default_value='false',
        description='Use digilab world instead of empty world')

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_dir, 'launch', 'launch_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
            'use_digilab': use_digilab
        }.items()
    )

    # Create a process that waits for Gazebo to be ready and then exits
    wait_for_gazebo = ExecuteProcess(
        cmd=['sleep', '10'],  # Wait 10 seconds for Gazebo to be fully ready
        output='screen'
    )

    # Launch robot spawning after Gazebo has started
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_dir, 'launch', 'spawn_robot_only.launch.py'
            ])
        ]),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'rviz': use_rviz,
            'use_digilab': use_digilab
        }.items()
    )

    # Event handler to spawn robot after Gazebo is ready
    spawn_robot_event = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_gazebo,
            on_exit=[robot_launch]
        )
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_use_digilab_cmd)

    # Add launch includes
    ld.add_action(gazebo_launch)
    ld.add_action(wait_for_gazebo)
    ld.add_action(spawn_robot_event)

    return ld
