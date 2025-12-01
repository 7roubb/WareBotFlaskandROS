#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import OccupancyGrid

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

from .mqtt_publisher import MQTTPublisher


class MapMerger(Node):
    """
    Single-robot map forwarder / merger.

    For your current setup:
      - Subscribes to /map (Nav2 static map)
      - Publishes /global_map
      - Sends same map to MQTT as JSON
    """

    def __init__(self):
        super().__init__("map_merger")

        # ======================================================
        # QoS profiles
        # ======================================================
        # Match Nav2 map_server's latched /map:
        #   - reliable
        #   - transient local (latched)
        self.map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Make /global_map also latched so RViz and others
        # get the latest map as soon as they subscribe
        self.global_map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ======================================================
        # Internal storage
        # ======================================================
        self.map_grid = None
        self.map_info = None

        # ======================================================
        # MQTT Publisher (to backend/dashboard)
        # ======================================================
        self.mqtt = MQTTPublisher(
            host="localhost",
            port=1883,
            topic="warehouse/map",  # backend listens on this
        )

        # ======================================================
        # ROS2 Publisher for Global Map (shared to all robots)
        # ======================================================
        self.global_map_pub = self.create_publisher(
            OccupancyGrid,
            "/global_map",
            self.global_map_qos,
        )

        # ======================================================
        # Subscription to /map
        # ======================================================
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            self.map_qos,
        )
        self.get_logger().info("📡 Subscribed to /map")
        self.get_logger().info("🚀 Single-Robot Map Merger Started")

        # Timer for publishing merged map
        self.timer = self.create_timer(0.5, self.publish_merged_map)

    # ============================================================
    # CALLBACK: STORE MAP
    # ============================================================
    def map_callback(self, msg: OccupancyGrid):
        try:
            expected = msg.info.width * msg.info.height
            if len(msg.data) != expected:
                self.get_logger().warn(
                    f"❌ Wrong map size from /map: expected {expected}, got {len(msg.data)}"
                )
                return

            grid = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height,
                msg.info.width,
            )

            self.map_grid = grid
            self.map_info = msg.info

            self.get_logger().info(
                f"✅ Received map from /map, size={grid.shape}"
            )

        except Exception as e:
            self.get_logger().error(f"[MAP CALLBACK ERROR] {e}")

    # ============================================================
    # MERGE & PUBLISH (MQTT + ROS2 Topic)
    # ============================================================
    def publish_merged_map(self):
        # For single robot, "merge" is just forwarding the map
        if self.map_grid is None or self.map_info is None:
            return

        try:
            base = self.map_grid.copy()
            info = self.map_info

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
                    "yaw": 0.0,
                },
                "data": merged,
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

            msg.data = [int(v) for v in merged]

            self.global_map_pub.publish(msg)

            self.get_logger().info(
                "🗺️ Published map → MQTT (warehouse/map) + ROS2 (/global_map)"
            )

        except Exception as e:
            self.get_logger().error(f"[MERGE ERROR] {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
