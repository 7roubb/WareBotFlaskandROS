from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="warebot_map_merger",
            executable="map_merger",
            name="map_merger",
            output="screen"
        )
    ])
