from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -------- Launch Arguments --------
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
        default_value="1883",
        description="MQTT broker port"
    )

    reference_frame_arg = DeclareLaunchArgument(
        "reference_frame",
        default_value="map",
        description="TF frame used for Nav2 goals"
    )

    attach_delay_arg = DeclareLaunchArgument(
        "attach_delay",
        default_value="2.0",
        description="Seconds robot waits to attach the shelf"
    )

    release_delay_arg = DeclareLaunchArgument(
        "release_delay",
        default_value="2.0",
        description="Seconds robot waits to release the shelf"
    )

    # -------- Task Runner Node --------
    node = Node(
        package="warebot_task_runner",
        executable="task_runner",
        name="task_runner",
        parameters=[
            {"robot_id": LaunchConfiguration("robot_id")},
            {"mqtt_host": LaunchConfiguration("mqtt_host")},
            {"mqtt_port": LaunchConfiguration("mqtt_port")},
            {"reference_frame": LaunchConfiguration("reference_frame")},
            {"attach_delay": LaunchConfiguration("attach_delay")},
            {"release_delay": LaunchConfiguration("release_delay")},
        ],
        output="screen",
        emulate_tty=True,   # يجعل اللوجز تظهر بكامل ألوانها
    )

    # -------- Final Launch Description --------
    return LaunchDescription([
        robot_id_arg,
        mqtt_host_arg,
        mqtt_port_arg,
        reference_frame_arg,
        attach_delay_arg,
        release_delay_arg,
        node,
    ])
