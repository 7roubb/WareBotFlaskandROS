"""
Launch file to start all WareBotFlask ROS2 nodes
"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for all WareBotFlask nodes"""
    
    return LaunchDescription([
        # Task Navigator Node - Main coordinator
        Node(
            package='warebot_task_navigator',
            executable='task_navigator',
            name='task_navigator',
            output='screen',
            parameters=[
                {'node_name': 'task_navigator'},
                {'control_loop_rate': 10},  # 10 Hz
                {'position_tolerance': 0.1},  # meters
                {'angle_tolerance': 0.1},  # radians
                {'max_velocity': 0.5},  # m/s
            ],
            remappings=[
                # Topic mappings from MQTT
                ('/task/assignment', '/task/assignment'),
                ('/robot/pose', '/robot/pose'),
                ('/reference_point/update', '/reference_point/update'),
                # Publisher topics
                ('/cmd_vel', '/cmd_vel'),
                ('/task/status', '/task/status'),
                ('/robot/state', '/robot/state'),
                ('/target_position', '/target_position'),
            ]
        ),
        
        # Task Executor Node - Movement execution
        Node(
            package='warebot_task_navigator',
            executable='task_executor',
            name='task_executor',
            output='screen',
            parameters=[
                {'node_name': 'task_executor'},
                {'execution_rate': 10},  # 10 Hz
                {'simulation_mode': True},  # Set to False for real hardware
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odometry', '/odometry'),
                ('/robot/pose', '/robot/pose'),
                ('/movement/status', '/movement/status'),
            ]
        ),
        
        # Reference Point Manager Node - Home/dock management
        Node(
            package='warebot_task_navigator',
            executable='reference_point_manager',
            name='reference_point_manager',
            output='screen',
            parameters=[
                {'node_name': 'reference_point_manager'},
                {'publication_rate': 1},  # 1 Hz
                {
                    'default_reference_points': {
                        'robot_1': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                        'robot_2': {'x': 2.0, 'y': 0.0, 'yaw': 0.0},
                        'robot_3': {'x': 4.0, 'y': 0.0, 'yaw': 0.0},
                    }
                }
            ],
            remappings=[
                ('/reference_point/command', '/reference_point/command'),
                ('/reference_point/update', '/reference_point/update'),
            ]
        ),
    ])
