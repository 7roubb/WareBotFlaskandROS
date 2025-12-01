from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    robots = DeclareLaunchArgument(
        "robots",
        default_value="robot1",
        description="Comma-separated list of robot names"
    )

    robots_cfg = LaunchConfiguration("robots")

    return LaunchDescription([
        robots,
        Node(
            package="warebot_map_merger",
            executable="map_merger",
            name="map_merger",
            output="screen",
            parameters=[{
                "robots": robots_cfg
            }]
        )
    ])
