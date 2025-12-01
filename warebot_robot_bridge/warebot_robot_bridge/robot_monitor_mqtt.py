#!/usr/bin/env python3
import json
import math
import psutil
import threading
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

import paho.mqtt.client as mqtt


class RobotMonitor(Node):
    """
    Reads robot telemetry (CPU, RAM, battery, temperature, pose)
    and publishes clean JSON to MQTT on:

      robots/mp400/<robot_name>/status
    """

    def __init__(self):
        super().__init__("robot_monitor")

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter("robot_name", "robot1")
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_username", "")
        self.declare_parameter("mqtt_password", "")

        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("pose_type", "amcl")   # amcl or odom
        self.declare_parameter("publish_rate_hz", 1.0)

        # ---------------------------
        # Load parameters
        # ---------------------------
        self.robot_name = self.get_parameter("robot_name").value
        self.mqtt_host = self.get_parameter("mqtt_host").value
        self.mqtt_port = self.get_parameter("mqtt_port").value
        self.mqtt_user = self.get_parameter("mqtt_username").value
        self.mqtt_pass = self.get_parameter("mqtt_password").value

        self.pose_topic = self.get_parameter("pose_topic").value
        self.pose_type = self.get_parameter("pose_type").value.lower()
        self.rate = self.get_parameter("publish_rate_hz").value

        # ---------------------------
        # Internal state
        # ---------------------------
        self.x = 0.0
        self.y = 0.0
        self.batt = 100.0
        self.temp = 40.0

        self._mqtt = None
        self._lock = threading.Lock()
        self._connected = False

        # ---------------------------
        # MQTT Setup
        # ---------------------------
        self._setup_mqtt()

        # ---------------------------
        # ROS Subscriptions
        # ---------------------------
        if self.pose_type == "amcl":
            self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self._pose_amcl,
                10,
            )
        else:
            self.create_subscription(
                Odometry,
                self.pose_topic,
                self._pose_odom,
                10,
            )

        self.create_subscription(
            BatteryState,
            "/battery_state",
            self._battery_cb,
            10,
        )

        # Timer to publish telemetry
        self.create_timer(1.0 / self.rate, self._publish)

        self.get_logger().info(f"Robot monitor started for {self.robot_name}")

    # ---------------------------
    # ROS Callbacks
    # ---------------------------
    def _pose_amcl(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def _pose_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def _battery_cb(self, msg):
        # If percentage 0→1 convert to %
        if msg.percentage is not None and msg.percentage > 0:
            self.batt = float(msg.percentage * 100.0)
        if msg.temperature > -273:
            self.temp = float(msg.temperature)

    # ---------------------------
    # MQTT Setup
    # ---------------------------
    def _setup_mqtt(self):
        self._mqtt = mqtt.Client(
            client_id=f"{self.robot_name}_monitor",
            protocol=mqtt.MQTTv5
        )

        if self.mqtt_user:
            self._mqtt.username_pw_set(self.mqtt_user, self.mqtt_pass)

        self._mqtt.on_connect = self._on_connect
        self._mqtt.on_disconnect = self._on_disconnect

        try:
            self._mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self._mqtt.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT connection error: {e}")

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        self._connected = True
        self.get_logger().info("[MQTT] Connected")

    def _on_disconnect(self, client, userdata, rc, properties=None):
        self._connected = False
        self.get_logger().warn("[MQTT] Disconnected")

    # ---------------------------
    # Publish Telemetry
    # ---------------------------
    def _publish(self):
        if not self._connected:
            return

        cpu = float(psutil.cpu_percent())
        ram = float(psutil.virtual_memory().percent)

        now = datetime.now(timezone.utc).isoformat()

        msg = {
            "cpu_usage": cpu,
            "ram_usage": ram,
            "battery_level": self.batt,
            "temperature": self.temp,
            "x": self.x,
            "y": self.y,
            "status": "IDLE",
            "timestamp": now,
        }

        topic = f"robots/mp400/{self.robot_name}/status"

        with self._lock:
            try:
                self._mqtt.publish(topic, json.dumps(msg))
            except Exception as e:
                self.get_logger().error(f"MQTT publish error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
