#!/usr/bin/env python3
"""
Launch file for MP-400 Footprint Switcher Node

Usage:
    # Launch with original footprint
    ros2 launch mp400_footprint_switcher footprint_switcher.launch.py initial_footprint:=original

    # Launch with extended footprint
    ros2 launch mp400_footprint_switcher footprint_switcher.launch.py initial_footprint:=extended

    # Launch with robot namespace
    ros2 launch mp400_footprint_switcher footprint_switcher.launch.py \
        robot_namespace:=robot1 initial_footprint:=original
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('mp400_footprint_switcher')
    
    # ========================================================================
    # LAUNCH ARGUMENTS
    # ========================================================================
    
    robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace (e.g., "robot1", "robot2")'
    )
    
    initial_footprint = DeclareLaunchArgument(
        'initial_footprint',
        default_value='original',
        choices=['original', 'extended'],
        description='Initial footprint configuration: "original" (0.30m x 0.30m) or "extended" (1.00m x 1.00m)'
    )
    
    update_frequency = DeclareLaunchArgument(
        'update_frequency',
        default_value='1.0',
        description='Frequency of footprint updates in Hz'
    )
    
    verbose = DeclareLaunchArgument(
        'verbose',
        default_value='True',
        description='Enable verbose logging'
    )
    
    # ========================================================================
    # FOOTPRINT SWITCHER NODE
    # ========================================================================
    
    footprint_switcher_node = Node(
        package='mp400_footprint_switcher',
        executable='footprint_switcher_node',
        name='mp400_footprint_switcher',
        namespace=LaunchConfiguration('robot_namespace'),
        output='screen',
        parameters=[
            {
                'robot_namespace': LaunchConfiguration('robot_namespace'),
                'initial_footprint': LaunchConfiguration('initial_footprint'),
                'update_frequency': LaunchConfiguration('update_frequency'),
                'verbose': LaunchConfiguration('verbose'),
            }
        ],
        emulate_tty=True,
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    
    ld = LaunchDescription([
        robot_namespace,
        initial_footprint,
        update_frequency,
        verbose,
        footprint_switcher_node,
    ])
    
    return ld
