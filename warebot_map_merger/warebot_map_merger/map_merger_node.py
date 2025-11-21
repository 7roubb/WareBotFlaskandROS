import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

import numpy as np
from .mqtt_publisher import MQTTPublisher


class MultiRobotMapMerger(Node):

    def __init__(self):
        super().__init__("multi_robot_map_merger")

        # Modify robot topics here
        self.robot_topics = [
            "/robot1/map",
            "/robot2/map",
            "/robot3/map",
        ]

        self.maps = {}
        self.map_headers = {}

        self.mqtt = MQTTPublisher(
            host="localhost",
            port=1883,
            topic="warehouse/map"
        )

        for t in self.robot_topics:
            self.create_subscription(
                OccupancyGrid,
                t,
                lambda msg, topic=t: self.map_callback(msg, topic),
                10
            )

        self.timer = self.create_timer(0.5, self.publish_merged_map)

        self.get_logger().info("Multi Robot Map Merger Started")

    def map_callback(self, msg, topic):
        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        self.maps[topic] = grid
        self.map_headers[topic] = msg.info

    def publish_merged_map(self):
        if len(self.maps) == 0:
            return

        first = next(iter(self.maps))
        base = self.maps[first].copy()
        info = self.map_headers[first]

        # Simple merge: take max cell value
        for topic, m in self.maps.items():
            if m.shape == base.shape:
                base = np.maximum(base, m)

        merged = base.flatten().tolist()

        map_json = {
            "width": info.width,
            "height": info.height,
            "resolution": info.resolution,
            "origin": {
                "x": info.origin.position.x,
                "y": info.origin.position.y,
                "yaw": 0.0
            },
            "data": merged
        }

        self.mqtt.publish_map(map_json)
        self.get_logger().info("Merged map published to MQTT")


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
