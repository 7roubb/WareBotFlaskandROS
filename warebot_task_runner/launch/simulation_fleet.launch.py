
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    # Perform substitutions
    num_robots_val = LaunchConfiguration('num_robots').perform(context)
    mqtt_host_val = LaunchConfiguration('mqtt_host').perform(context)
    mqtt_port_val = LaunchConfiguration('mqtt_port').perform(context)
    enable_hardware_val = LaunchConfiguration('enable_hardware').perform(context)
    
    try:
        num_robots = int(num_robots_val)
    except ValueError:
        num_robots = 4  # Default fallback
        print(f"Warning: Invalid num_robots value '{num_robots_val}', using default 4")

    actions = []

    for i in range(num_robots):
        robot_name = f"robot{i}"
        ns = robot_name  # Namespace if needed
        
        # 1. Robot Monitor Node
        monitor_node = Node(
            package="warebot_task_runner",
            executable="robot_monitor",
            name=f"{robot_name}_monitor",
            namespace=ns,
            output="screen",
            parameters=[
                {"robot_name": robot_name},
                {"mqtt_host": mqtt_host_val},
                {"mqtt_port": mqtt_port_val},
                {"enable_task_monitoring": True},
                {"task_status_topic": f"/{robot_name}/task/status"},
                {"pose_type": "amcl"},
                {"pose_topic": f"/{robot_name}/amcl_pose"},
                {"enable_lidar_monitoring": True},
                {"lidar_topic": f"/{robot_name}/scan"},
                {"enable_velocity_monitoring": True},
                {"velocity_topic": f"/{robot_name}/cmd_vel"},
            ]
        )
        
        # 2. Task Runner Node (Simulation Mode)
        # Note: In simulation, we disable hardware so mock controllers are used.
        # But we still need valid topics for navigation etc.
        task_runner_node = Node(
            package="warebot_task_runner",
            executable="task_runner",
            name="task_runner", # Name will be Namespaced
            namespace=ns,
            output="screen",
            parameters=[
                {"robot_id": robot_name},
                {"mqtt_host": mqtt_host_val},
                {"mqtt_port": mqtt_port_val},
                {"enable_hardware": enable_hardware_val == 'true'},
                {"cmd_vel_topic": f"/{robot_name}/cmd_vel"},
                {"estop_topic": f"/{robot_name}/emergency_stop_state"},
                {"image_topic": f"/{robot_name}/image_raw"},
                # Empty calibration path triggers mock or default behavior in sim
                {"calibration_file_path": ""}, 
            ]
        )

        actions.append(monitor_node)
        actions.append(task_runner_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_robots',
            default_value='4',
            description='Number of robots to launch components for'
        ),
        DeclareLaunchArgument(
            'mqtt_host',
            default_value='localhost',
            description='MQTT Host'
        ),
        DeclareLaunchArgument(
            'mqtt_port',
            default_value='1884',
            description='MQTT Port'
        ),
        DeclareLaunchArgument(
            'enable_hardware',
            default_value='false',
            description='Enable hardware (set to false for simulation)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
