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

    @staticmethod
    def _quaternion_to_yaw(qx, qy, qz, qw):
        """Convert quaternion to yaw (rotation around z-axis) in radians."""
        sinr_cosp = 2 * (qw * qz + qx * qy)
        cosr_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(sinr_cosp, cosr_cosp)
        return yaw

    def __init__(self):
        super().__init__("robot_monitor")

        # ---------------------------
        # Parameters
        # ---------------------------
        self.declare_parameter("robot_name", "robot1")
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 8883)
        self.declare_parameter("mqtt_username", "")
        self.declare_parameter("mqtt_password", "")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("pose_type", "amcl")
        self.declare_parameter("publish_rate_hz", 1.0)
        
        # MP-400 specific parameters
        self.declare_parameter("enable_velocity_monitoring", False)
        self.declare_parameter("velocity_topic", "/cmd_vel")
        self.declare_parameter("enable_lidar_monitoring", False)
        self.declare_parameter("lidar_topic", "/scan")
        self.declare_parameter("enable_task_monitoring", False)
        self.declare_parameter("task_status_topic", "/task/status")
        self.declare_parameter("cpu_threshold", 80.0)
        self.declare_parameter("temp_threshold", 70.0)
        self.declare_parameter("battery_low_threshold", 20.0)

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
        
        # MP-400 specific params
        self.enable_velocity = self.get_parameter("enable_velocity_monitoring").value
        self.velocity_topic = self.get_parameter("velocity_topic").value
        self.enable_lidar = self.get_parameter("enable_lidar_monitoring").value
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.enable_task_monitoring = self.get_parameter("enable_task_monitoring").value
        self.task_status_topic = self.get_parameter("task_status_topic").value
        self.cpu_threshold = self.get_parameter("cpu_threshold").value
        self.temp_threshold = self.get_parameter("temp_threshold").value
        self.batt_low_threshold = self.get_parameter("battery_low_threshold").value

        # ---------------------------
        # Internal state
        # ---------------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.batt = 100.0
        self.temp = 40.0
        self.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self.is_charging = False
        
        # Additional state
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.min_lidar_range = 0.0
        self.last_lidar_timestamp = None
        self.lidar_data_changing = False
        self.previous_lidar_ranges = None
        self.has_active_task = False
        self.current_task_state = "IDLE"
        
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
        
        # Optional velocity monitoring
        if self.enable_velocity:
            from geometry_msgs.msg import Twist
            self.create_subscription(
                Twist,
                self.velocity_topic,
                self._velocity_cb,
                10,
            )
        
        # Optional lidar monitoring
        if self.enable_lidar:
            from sensor_msgs.msg import LaserScan
            self.create_subscription(
                LaserScan,
                self.lidar_topic,
                self._lidar_cb,
                10,
            )
        
        # Optional task state monitoring
        if self.enable_task_monitoring:
            self.create_subscription(
                String,
                self.task_status_topic,
                self._task_status_cb,
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
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = self._quaternion_to_yaw(qx, qy, qz, qw)

    def _pose_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = self._quaternion_to_yaw(qx, qy, qz, qw)

    def _battery_cb(self, msg):
        if msg.percentage is not None and msg.percentage > 0:
            self.batt = float(msg.percentage * 100.0)
        if msg.temperature > -273:
            self.temp = float(msg.temperature)
        
        self.power_supply_status = msg.power_supply_status
        self.is_charging = (
            self.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        )
    
    def _velocity_cb(self, msg):
        """Track robot velocity"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def _lidar_cb(self, msg):
        """Track minimum lidar range and detect movement from changing lidar data"""
        import time
        
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.min_lidar_range = min(valid_ranges) if valid_ranges else 0.0
        
        # Detect if robot is moving by checking if lidar readings are changing
        current_time = time.time()
        
        if self.previous_lidar_ranges is not None and len(valid_ranges) > 0:
            # Compare current lidar data with previous
            # If readings change significantly, robot is likely moving
            if len(self.previous_lidar_ranges) == len(valid_ranges):
                # Calculate difference between current and previous readings
                differences = [abs(a - b) for a, b in zip(valid_ranges, self.previous_lidar_ranges)]
                avg_difference = sum(differences) / len(differences) if differences else 0
                
                # If average difference > threshold, robot is moving
                # Threshold of 0.05m means robot detected movement
                self.lidar_data_changing = avg_difference > 0.05
            else:
                # Different number of valid points might indicate movement
                self.lidar_data_changing = True
        
        # Store current readings for next comparison
        self.previous_lidar_ranges = valid_ranges.copy() if valid_ranges else None
        self.last_lidar_timestamp = current_time
    
    def _task_status_cb(self, msg):
        """Track task status to determine if robot is busy"""
        try:
            task_data = json.loads(msg.data)
            task_status = task_data.get("status", "IDLE")
            
            # States that indicate robot is busy with a task
            BUSY_STATES = [
                "ASSIGNED", "MOVING_TO_PICKUP", "ALIGNING", "ALIGNED",
                "MOVING_TO_DROP", "ARRIVED_AT_DROP", "RELEASED",
                "MOVING_TO_REFERENCE"
            ]
            
            self.current_task_state = task_status
            self.has_active_task = task_status in BUSY_STATES
            
        except Exception as e:
            self.get_logger().debug(f"Failed to parse task status: {e}")

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
    # Determine Robot Status
    # ---------------------------
    def _get_robot_status(self):
        """Determine current robot status based on telemetry."""
        # Priority: ERROR > CHARGING > BUSY > IDLE
        
        # Check for errors
        cpu = psutil.cpu_percent()
        if cpu > self.cpu_threshold or self.temp > self.temp_threshold:
            return "ERROR"
        
        # Check if charging
        if self.is_charging:
            return "CHARGING"
        
        # Check if robot has an active task (busy with task execution)
        if self.has_active_task:
            return "BUSY"
        
        # Check if moving based on velocity
        if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
            return "BUSY"
        
        # Check if moving based on changing lidar data
        if self.enable_lidar and self.lidar_data_changing:
            return "BUSY"
        
        return "IDLE"

    # ---------------------------
    # Publish Telemetry
    # ---------------------------
    def _publish(self):
        if not self._connected:
            return

        cpu = float(psutil.cpu_percent())
        ram = float(psutil.virtual_memory().percent)
        now = datetime.now(timezone.utc).isoformat()
        status = self._get_robot_status()

        msg = {
            "cpu_usage": cpu,
            "ram_usage": ram,
            "battery_level": self.batt,
            "temperature": self.temp,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "status": status,
            "is_charging": self.is_charging,
            "timestamp": now,
        }
        
        # Add optional fields if enabled
        if self.enable_velocity:
            msg["linear_velocity"] = self.linear_vel
            msg["angular_velocity"] = self.angular_vel
        
        if self.enable_lidar:
            msg["min_obstacle_distance"] = self.min_lidar_range
            msg["lidar_detecting_movement"] = self.lidar_data_changing
        
        if self.enable_task_monitoring:
            msg["task_state"] = self.current_task_state
            msg["has_active_task"] = self.has_active_task

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