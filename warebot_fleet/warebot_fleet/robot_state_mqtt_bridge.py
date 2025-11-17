#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import json

class RobotStateMQTT(Node):

    def __init__(self):
        super().__init__("robot_state_mqtt_bridge")

        self.robot_name = self.declare_parameter("robot_name", "robot1").value
        self.mqtt_host = self.declare_parameter("mqtt_host", "localhost").value
        self.mqtt_port = self.declare_parameter("mqtt_port", 1883).value

        self.client = mqtt.Client(client_id=f"{self.robot_name}_bridge")
        self.client.connect(self.mqtt_host, self.mqtt_port, 60)

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.get_logger().info(f"[MQTT Bridge] Started for {self.robot_name}")

    def odom_callback(self, msg: Odometry):

        payload = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "cpu_usage": 30,
            "ram_usage": 40,
            "battery_level": 85,
            "temperature": 35.2,
            "status": "IDLE",
        }

        topic = f"robots/mp400/{self.robot_name}/status"
        self.client.publish(topic, json.dumps(payload))

        self.get_logger().info(f"[MQTT] → {topic}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMQTT()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
