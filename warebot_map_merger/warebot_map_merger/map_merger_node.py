#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import OccupancyGrid

from .mqtt_publisher import MQTTPublisher


class MultiRobotMapMerger(Node):

    def __init__(self):
        super().__init__("multi_robot_map_merger")

        # ======================
        # Parameters
        # ======================
        # robots: comma-separated list, e.g. "robot1,robot2"
        self.declare_parameter("robots", "robot1")
        robots_param = self.get_parameter("robots").value
        self.robot_list = [r.strip() for r in robots_param.split(",") if r.strip()]

        if not self.robot_list:
            self.robot_list = ["robot1"]

        self.get_logger().info(f"🤖 Robots list: {self.robot_list}")

        # ======================
        # Prepare topics
        # ======================
        self.robot_topics = [f"/{r}/map" for r in self.robot_list]

        # Store latest map per robot
        self.maps = {}
        self.map_info = {}

        # ======================
        # MQTT Publisher (to backend/dashboard)
        # ======================
        self.mqtt = MQTTPublisher(
            host="localhost",
            port=1883,
            topic="warehouse/map"  # backend listens on this
        )

        # ======================
        # ROS2 Publisher for Global Map (shared to all robots)
        # ======================
        self.global_map_pub = self.create_publisher(
            OccupancyGrid,
            "/global_map",
            10
        )

        # ======================
        # Subscriptions
        # ======================
        for topic in self.robot_topics:
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, t=topic: self.map_callback(msg, t),
                10
            )
            self.get_logger().info(f"📡 Subscribed to local map: {topic}")

        # Timer for merging + publishing
        self.timer = self.create_timer(0.5, self.publish_merged_map)
        self.get_logger().info("🚀 Multi-Robot Map Merger Started")

    # ============================================================
    # CALLBACK: STORE MAP FOR EACH ROBOT
    # ============================================================
    def map_callback(self, msg: OccupancyGrid, topic: str):
        try:
            expected = msg.info.width * msg.info.height
            if len(msg.data) != expected:
                self.get_logger().warn(
                    f"❌ Wrong map size from {topic}: expected {expected}, got {len(msg.data)}"
                )
                return

            grid = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height,
                msg.info.width
            )

            self.maps[topic] = grid
            self.map_info[topic] = msg.info

        except Exception as e:
            self.get_logger().error(f"[MAP CALLBACK ERROR] {e}")

    # ============================================================
    # MERGE & PUBLISH (MQTT + ROS2 Topic)
    # ============================================================
    def publish_merged_map(self):
        if len(self.maps) == 0:
            return

        try:
            # Use first map as base
            first_topic = next(iter(self.maps))
            base = self.maps[first_topic].copy()
            info = self.map_info[first_topic]

            # Merge via max (occupancy OR)
            for topic, m in self.maps.items():
                if m.shape != base.shape:
                    self.get_logger().warn(
                        f"⚠️ Skipping {topic}: size mismatch {m.shape} vs {base.shape}"
                    )
                    continue
                base = np.maximum(base, m)

            merged = base.flatten().tolist()

            # ===================================================
            # 1) SEND TO BACKEND VIA MQTT (JSON)
            # ===================================================
            payload = {
                "width": int(info.width),
                "height": int(info.height),
                "resolution": float(info.resolution),
                "origin": {
                    "x": float(info.origin.position.x),
                    "y": float(info.origin.position.y),
                    "yaw": 0.0
                },
                "data": merged
            }

            self.mqtt.publish_map(payload)

            # ===================================================
            # 2) PUBLISH GLOBAL MAP AS ROS2 TOPIC (/global_map)
            # ===================================================
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            msg.info.width = info.width
            msg.info.height = info.height
            msg.info.resolution = info.resolution
            msg.info.origin = info.origin  # reuse original origin

            # Ensure int list
            msg.data = [int(v) for v in merged]

            self.global_map_pub.publish(msg)

            self.get_logger().info("🗺️ Published merged map → MQTT (warehouse/map) + ROS2 (/global_map)")

        except Exception as e:
            self.get_logger().error(f"[MERGE ERROR] {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
