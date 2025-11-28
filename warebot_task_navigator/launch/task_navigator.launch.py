"""
Launch file for task navigator nodes
Launches: task_navigator, task_executor, and reference_point_manager nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='localhost',
        description='MQTT broker address'
    )
    
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_1',
        description='Robot ID for task assignment'
    )

    # Get launch arguments
    mqtt_broker = LaunchConfiguration('mqtt_broker')
    mqtt_port = LaunchConfiguration('mqtt_port')
    robot_id = LaunchConfiguration('robot_id')

    # Task Navigator Node - Main coordinator
    task_navigator_node = Node(
        package='warebot_task_navigator',
        executable='task_navigator',
        name='task_navigator',
        output='screen',
        parameters=[
            {'mqtt_broker': mqtt_broker},
            {'mqtt_port': mqtt_port},
            {'use_sim_time': False},
            {'control_frequency': 10.0},  # 10Hz control loop
            {'distance_tolerance': 0.1},   # 0.1m tolerance
            {'angle_tolerance': 0.1},      # 0.1 rad tolerance
            {'max_linear_speed': 0.5},     # m/s
            {'max_angular_speed': 0.5},    # rad/s
        ],
        remappings=[
            ('task_assignment', '/ros2/task/assignment'),
            ('robot_pose', '/robot/pose'),
            ('reference_point_update', '/reference_point/update'),
            ('task_status_pub', '/task/status'),
            ('cmd_vel_pub', '/cmd_vel'),
            ('target_pub', '/target_position'),
            ('robot_state_pub', '/robot/state'),
        ]
    )

    # Task Executor Node - Velocity execution
    task_executor_node = Node(
        package='warebot_task_navigator',
        executable='task_executor',
        name='task_executor',
        output='screen',
        parameters=[
            {'mqtt_broker': mqtt_broker},
            {'mqtt_port': mqtt_port},
            {'robot_id': robot_id},
            {'use_sim_time': False},
            {'execution_frequency': 10.0},  # 10Hz execution loop
            {'robot_mass': 50.0},            # kg
            {'max_linear_accel': 0.5},       # m/s²
            {'max_angular_accel': 0.5},      # rad/s²
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odometry', '/odometry'),
            ('pose_pub', '/robot/pose'),
            ('status_pub', '/movement/status'),
        ]
    )

    # Reference Point Manager Node - Home/dock locations
    reference_point_manager_node = Node(
        package='warebot_task_navigator',
        executable='reference_point_manager',
        name='reference_point_manager',
        output='screen',
        parameters=[
            {'mqtt_broker': mqtt_broker},
            {'mqtt_port': mqtt_port},
            {'use_sim_time': False},
            {'publish_frequency': 1.0},  # 1Hz publication
            {'default_reference_points': [
                {'robot_id': 'robot_1', 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'description': 'Robot 1 Dock'},
                {'robot_id': 'robot_2', 'x': 2.0, 'y': 0.0, 'yaw': 0.0, 'description': 'Robot 2 Dock'},
                {'robot_id': 'robot_3', 'x': 4.0, 'y': 0.0, 'yaw': 0.0, 'description': 'Robot 3 Dock'},
            ]},
        ],
        remappings=[
            ('reference_point_command', '/ros2/reference_point/command'),
            ('reference_point_pub', '/reference_point/update'),
        ]
    )

    # Create launch description with all nodes
    ld = LaunchDescription([
        # Launch arguments
        mqtt_broker_arg,
        mqtt_port_arg,
        robot_id_arg,
        
        # Nodes
        task_navigator_node,
        task_executor_node,
        reference_point_manager_node,
    ])

    return ld
