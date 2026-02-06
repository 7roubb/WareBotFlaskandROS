#!/usr/bin/env python3
"""
Task Runner Launch File - CORRECTED VERSION
Properly launches task runner in robot namespace to access Nav2 action server
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get defaults from environment
    default_robot_id = os.environ.get('ROBOT_ID', 'robot0')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'robot_id',
            default_value=default_robot_id,
            description='Robot ID (must match namespace used in navigation, e.g., robot0, robot1)'
        ),
        DeclareLaunchArgument(
            'mqtt_host',
            default_value='localhost',
            description='MQTT broker hostname'
        ),
        DeclareLaunchArgument(
            'mqtt_port',
            default_value='1884',
            description='MQTT broker port'
        ),
        DeclareLaunchArgument(
            'mqtt_qos',
            default_value='1',
            description='MQTT QoS level'
        ),
        DeclareLaunchArgument(
            'initial_footprint',
            default_value='original',
            description='Initial robot footprint (original or extended)'
        ),
        
        # Task Runner Node
        # CRITICAL: Must be in same namespace as Nav2 to access action server
        Node(
            package='warebot_task_runner',
            executable='task_runner',
            name='task_runner',
            namespace=LaunchConfiguration('robot_id'),  # ← This puts it in /robot0, /robot1, etc.
            output='screen',
            parameters=[
                {'robot_id': LaunchConfiguration('robot_id')},
                {'mqtt_host': LaunchConfiguration('mqtt_host')},
                {'mqtt_port': LaunchConfiguration('mqtt_port')},
                {'mqtt_qos': LaunchConfiguration('mqtt_qos')},
                {'robot_namespace': LaunchConfiguration('robot_id')},
                {'initial_footprint': LaunchConfiguration('initial_footprint')},
                {'use_sim_time': True},
            ],
        ),
    ])