#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # -------------------------
    # Declare parameters
    # -------------------------
    robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="robot1",
        description="Robot name used in MQTT topic"
    )

    mqtt_host = DeclareLaunchArgument(
        "mqtt_host",
        default_value="localhost",
        description="MQTT broker host"
    )

    mqtt_port = DeclareLaunchArgument(
        "mqtt_port",
        default_value="1883",
        description="MQTT broker port"
    )

    mqtt_username = DeclareLaunchArgument(
        "mqtt_username",
        default_value="",
        description="MQTT username"
    )

    mqtt_password = DeclareLaunchArgument(
        "mqtt_password",
        default_value="",
        description="MQTT password"
    )

    pose_topic = DeclareLaunchArgument(
        "pose_topic",
        default_value="/amcl_pose",
        description="Pose topic (amcl or odom)"
    )

    pose_type = DeclareLaunchArgument(
        "pose_type",
        default_value="amcl",
        description="Type of pose source: amcl or odom"
    )

    publish_rate = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="1.0",
        description="Rate of telemetry publishing"
    )

    # -------------------------
    # Create the Node
    # -------------------------
    monitor_node = Node(
        package="warebot_robot_bridge",
        executable="robot_monitor_mqtt",
        name="robot_monitor",
        output="screen",
        parameters=[
            {"robot_name": LaunchConfiguration("robot_name")},
            {"mqtt_host": LaunchConfiguration("mqtt_host")},
            {"mqtt_port": LaunchConfiguration("mqtt_port")},
            {"mqtt_username": LaunchConfiguration("mqtt_username")},
            {"mqtt_password": LaunchConfiguration("mqtt_password")},
            {"pose_topic": LaunchConfiguration("pose_topic")},
            {"pose_type": LaunchConfiguration("pose_type")},
            {"publish_rate_hz": LaunchConfiguration("publish_rate_hz")},
        ]
    )

    # -------------------------
    # Return LaunchDescription
    # -------------------------
    return LaunchDescription([
        robot_name,
        mqtt_host,
        mqtt_port,
        mqtt_username,
        mqtt_password,
        pose_topic,
        pose_type,
        publish_rate,
        monitor_node
    ])
