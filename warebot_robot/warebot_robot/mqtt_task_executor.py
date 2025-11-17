#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json

from geometry_msgs.msg import PoseStamped

class MQTTTaskExecutor(Node):

    def __init__(self):
        super().__init__("mqtt_task_executor")

        self.robot_name = self.declare_parameter("robot_name", "robot1").value
        self.mqtt_host = self.declare_parameter("mqtt_host", "localhost").value
        self.mqtt_port = self.declare_parameter("mqtt_port", 1883).value

        self.publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.client = mqtt.Client(client_id=f"{self.robot_name}_task_exec")
        self.client.on_message = self.on_mqtt_message
        self.client.connect(self.mqtt_host, self.mqtt_port, 60)

        task_topic = f"robots/mp400/{self.robot_name}/task"
        self.client.subscribe(task_topic)

        self.client.loop_start()

        self.get_logger().info(f"[TaskExec] Listening on {task_topic}")

    def on_mqtt_message(self, client, userdata, msg):
        payload = json.loads(msg.payload.decode())

        shelf_id = payload.get("shelf_id")
        task_id = payload.get("task_id")

        self.get_logger().info(f"[TaskExec] Received task → shelf {shelf_id}")

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 1.0
        goal.pose.position.y = 1.0
        goal.pose.orientation.w = 1.0

        self.publisher.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = MQTTTaskExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
