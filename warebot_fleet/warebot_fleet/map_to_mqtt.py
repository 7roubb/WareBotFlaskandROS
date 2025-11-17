#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import paho.mqtt.client as mqtt
import json

class MapToMQTT(Node):
    def __init__(self):
        super().__init__("map_to_mqtt")

        self.mqtt_host = self.declare_parameter("mqtt_host", "localhost").value
        self.mqtt_port = self.declare_parameter("mqtt_port", 1883).value

        self.client = mqtt.Client(client_id="map_publisher")
        self.client.connect(self.mqtt_host, self.mqtt_port, 60)

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)

    def map_callback(self, msg: OccupancyGrid):

        payload = {
            "resolution": msg.info.resolution,
            "width": msg.info.width,
            "height": msg.info.height,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
            },
            "data": list(msg.data)
        }

        self.client.publish("warehouse/map", json.dumps(payload))
        self.get_logger().info(f"[MQTT] Published /map ({msg.info.width}x{msg.info.height})")

def main(args=None):
    rclpy.init(args=args)
    node = MapToMQTT()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
