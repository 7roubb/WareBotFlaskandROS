from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Arguments
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="robot_1",
        description="Unique ID of the robot"
    )

    mqtt_host_arg = DeclareLaunchArgument(
        "mqtt_host",
        default_value="localhost",
        description="MQTT broker hostname"
    )

    mqtt_port_arg = DeclareLaunchArgument(
        "mqtt_port",
        default_value="1884",
        description="MQTT broker port"
    )
    
    esp32_port_arg = DeclareLaunchArgument(
        "esp32_port",
        default_value="/dev/ttyUSB0",
        description="ESP32 serial port"
    )
    
    use_path_following_arg = DeclareLaunchArgument(
        "use_path_following",
        default_value="true",
        description="Use FollowPath action (true) or NavigateToPose (false)"
    )

    # Task Runner Node
    node = Node(
        package="warebot_task_runner",
        executable="task_runner",
        name="task_runner",
        parameters=[
            {"robot_id": LaunchConfiguration("robot_id")},
            {"mqtt_host": LaunchConfiguration("mqtt_host")},
            {"mqtt_port": LaunchConfiguration("mqtt_port")},
            {"esp32_port": LaunchConfiguration("esp32_port")},
            {"esp32_baudrate": 115200},
            {"actuator_extend_time": 22.0},
            {"actuator_retract_time": 22.0},
            {"backend_enabled": True},
            {"backend_host": "localhost"},
            {"backend_port": 5000},
            {"use_path_following": LaunchConfiguration("use_path_following")}
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_id_arg,
        mqtt_host_arg,
        mqtt_port_arg,
        esp32_port_arg,
        use_path_following_arg,
        node,
    ])