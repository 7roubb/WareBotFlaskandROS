#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import OccupancyGrid

from .mqtt_publisher import MQTTPublisher


class MultiRobotMapMerger(Node):

    def __init__(self):
        super().__init__("multi_robot_map_merger")

        self.robot_topics = [
            "/robot1/map",
            # "/robot2/map",  # Add more when needed
            # "/robot3/map",
        ]

        self.maps = {}
        self.map_info = {}

        self.mqtt = MQTTPublisher(
            host="localhost",
            port=1883,
            topic="warehouse/map"
        )

        for topic in self.robot_topics:
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, t=topic: self.map_callback(msg, t),
                10
            )

        self.timer = self.create_timer(0.5, self.publish_merged_map)
        self.get_logger().info("🚀 Multi-Robot Map Merger Started")

    def map_callback(self, msg, topic):
        expected_size = msg.info.width * msg.info.height

        if len(msg.data) != expected_size:
            self.get_logger().error(
                f"❌ Map size mismatch from {topic}: expected {expected_size}, but got {len(msg.data)}"
            )
            return

        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height,
            msg.info.width
        )

        self.maps[topic] = grid
        self.map_info[topic] = msg.info

    def publish_merged_map(self):
        if len(self.maps) == 0:
            return

        first_topic = next(iter(self.maps))
        base = self.maps[first_topic].copy()
        info = self.map_info[first_topic]

        for topic, m in self.maps.items():
            if m.shape != base.shape:
                self.get_logger().warn(f"⚠️ Skipping {topic} — different map size!")
                continue

            base = np.maximum(base, m)

        merged = base.flatten().tolist()

        payload = {
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

        self.mqtt.publish_map(payload)
        self.get_logger().info("🗺️ Published merged map to MQTT → warehouse/map")

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
