#!/usr/bin/env python3
"""
Enhanced WareBot Task Runner Node with Path Following, AprilTag Alignment & ESP32
"""

import json
import math
import time
import threading
from enum import Enum
from typing import Optional, Dict, Any, List, Tuple

import requests
import serial
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose, FollowPath
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
import paho.mqtt.client as mqtt


class TaskState(Enum):
    """Task execution states"""
    IDLE = "IDLE"
    ASSIGNED = "ASSIGNED"
    MOVING_TO_PICKUP = "MOVING_TO_PICKUP"
    ARRIVED_AT_PICKUP = "ARRIVED_AT_PICKUP"
    ALIGNING_TAG = "ALIGNING_TAG"
    ALIGNED = "ALIGNED"
    EXTENDING_ACTUATORS = "EXTENDING_ACTUATORS"
    ATTACHED = "ATTACHED"
    MOVING_TO_DROP = "MOVING_TO_DROP"
    ARRIVED_AT_DROP = "ARRIVED_AT_DROP"
    RETRACTING_ACTUATORS = "RETRACTING_ACTUATORS"
    RELEASED = "RELEASED"
    MOVING_TO_REFERENCE = "MOVING_TO_REFERENCE"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"


class RobotTaskRunner(Node):
    """Main task runner node with path following, AprilTag alignment and ESP32 control"""

    def __init__(self):
        super().__init__("task_runner")

        # --- ROS2 parameters ---
        self.declare_parameter("robot_id", "robot_1")
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1884)
        self.declare_parameter("mqtt_qos", 1)
        self.declare_parameter("mqtt_reconnect_base", 1.0)
        self.declare_parameter("mqtt_reconnect_max", 30.0)
        self.declare_parameter("backend_host", "localhost")
        self.declare_parameter("backend_port", 5000)
        self.declare_parameter("backend_enabled", True)
        self.declare_parameter("use_path_following", True)  # Use FollowPath vs NavigateToPose
        
        # ESP32 serial parameters
        self.declare_parameter("esp32_port", "/dev/ttyUSB0")
        self.declare_parameter("esp32_baudrate", 115200)
        self.declare_parameter("actuator_extend_time", 22.0)
        self.declare_parameter("actuator_retract_time", 22.0)

        self.robot_id = self.get_parameter("robot_id").value
        self.mqtt_host = self.get_parameter("mqtt_host").value
        self.mqtt_port = int(self.get_parameter("mqtt_port").value)
        self.mqtt_qos = int(self.get_parameter("mqtt_qos").value)
        self._reconnect_base = float(self.get_parameter("mqtt_reconnect_base").value)
        self._reconnect_max = float(self.get_parameter("mqtt_reconnect_max").value)
        self.backend_host = self.get_parameter("backend_host").value
        self.backend_port = int(self.get_parameter("backend_port").value)
        self.backend_enabled = self.get_parameter("backend_enabled").value
        self.use_path_following = self.get_parameter("use_path_following").value
        
        # ESP32 parameters
        self.esp32_port = self.get_parameter("esp32_port").value
        self.esp32_baudrate = int(self.get_parameter("esp32_baudrate").value)
        self.actuator_extend_time = float(self.get_parameter("actuator_extend_time").value)
        self.actuator_retract_time = float(self.get_parameter("actuator_retract_time").value)

        self.get_logger().info(f"Task Runner initialized for {self.robot_id}")
        self.get_logger().info(f"Path following mode: {'FollowPath' if self.use_path_following else 'NavigateToPose'}")
        if self.backend_enabled:
            self.get_logger().info(f"Backend API: {self.backend_host}:{self.backend_port}")

        # --- State ---
        self.current_task: Optional[Dict[str, Any]] = None
        self.task_state = TaskState.IDLE
        self.reference_point: Optional[Dict[str, float]] = None

        # --- ESP32 Serial Connection ---
        self.esp32_serial = None
        self._init_esp32_serial()

        # --- Nav2 action clients ---
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._path_client = ActionClient(self, FollowPath, "follow_path")
        
        self.get_logger().info("Waiting for Nav2 servers...")
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("NavigateToPose server not available yet. Continuing...")
        else:
            self.get_logger().info("NavigateToPose server ready")
            
        if self.use_path_following:
            if not self._path_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().warn("FollowPath server not available yet. Continuing...")
            else:
                self.get_logger().info("FollowPath server ready")

        # --- AprilTag Alignment Service Client ---
        self._alignment_client = self.create_client(SetBool, '/apriltag/enable_alignment')
        self.get_logger().info("Waiting for AprilTag alignment service...")
        if not self._alignment_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("AprilTag service not available yet. Continuing...")
        else:
            self.get_logger().info("AprilTag alignment service ready")

        # --- AprilTag Alignment Status Subscriber ---
        self.alignment_status_sub = self.create_subscription(
            String,
            '/apriltag/alignment_status',
            self.alignment_status_callback,
            10
        )
        self.alignment_status = "UNKNOWN"

        # --- ROS2 publisher ---
        self.task_status_pub = self.create_publisher(String, f"/{self.robot_id}/task/status", 10)

        # --- Robot position tracking ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_position_update = time.time()
        self._position_lock = threading.Lock()
        self.position_history = []

        # --- MQTT client setup ---
        try:
            self.mqtt_client = mqtt.Client(
                client_id=self.robot_id,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1
            )
        except Exception as e:
            self.get_logger().warn(f"mqtt.Client init fallback: {e}")
            self.mqtt_client = mqtt.Client(client_id=self.robot_id)

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect

        self._mqtt_connected = False
        self._mqtt_reconnect_attempts = 0
        self._mqtt_lock = threading.Lock()

        self._start_mqtt()

        self.get_logger().info("=" * 60)
        self.get_logger().info("🤖 TASK RUNNER READY - Waiting for tasks")
        self.get_logger().info("=" * 60)

    # ============================================================
    # ESP32 Serial Communication
    # ============================================================

    def _init_esp32_serial(self):
        """Initialize serial connection to ESP32"""
        try:
            self.esp32_serial = serial.Serial(
                port=self.esp32_port,
                baudrate=self.esp32_baudrate,
                timeout=1.0
            )
            time.sleep(2.0)  # Wait for ESP32 to reset
            self.get_logger().info(f"✅ ESP32 connected on {self.esp32_port}")
            
            # Read welcome message
            while self.esp32_serial.in_waiting > 0:
                line = self.esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.get_logger().info(f"ESP32: {line}")
                    
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to ESP32: {e}")
            self.esp32_serial = None

    def send_esp32_command(self, command: str) -> bool:
        """Send command to ESP32 and wait for response"""
        if not self.esp32_serial or not self.esp32_serial.is_open:
            self.get_logger().error("ESP32 serial not connected!")
            return False

        try:
            cmd = f"{command}\n"
            self.esp32_serial.write(cmd.encode('utf-8'))
            self.get_logger().info(f"📤 Sent to ESP32: {command}")
            
            # Read response
            time.sleep(0.5)
            while self.esp32_serial.in_waiting > 0:
                response = self.esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    self.get_logger().info(f"📥 ESP32: {response}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to send command to ESP32: {e}")
            return False

    def extend_actuators(self):
        """Extend linear actuators and wait for completion"""
        self.get_logger().info("🔧 Extending linear actuators...")
        self.task_state = TaskState.EXTENDING_ACTUATORS
        self.publish_status("EXTENDING_ACTUATORS")
        
        if self.send_esp32_command("EXTEND"):
            self.get_logger().info(f"⏳ Waiting {self.actuator_extend_time}s for extension...")
            time.sleep(self.actuator_extend_time)
            
            self.send_esp32_command("STOP")
            self.get_logger().info("✅ Actuators extended - Shelf attached")
            return True
        else:
            self.get_logger().error("❌ Failed to extend actuators")
            return False

    def retract_actuators(self):
        """Retract linear actuators and wait for completion"""
        self.get_logger().info("🔧 Retracting linear actuators...")
        self.task_state = TaskState.RETRACTING_ACTUATORS
        self.publish_status("RETRACTING_ACTUATORS")
        
        if self.send_esp32_command("RETRACT"):
            self.get_logger().info(f"⏳ Waiting {self.actuator_retract_time}s for retraction...")
            time.sleep(self.actuator_retract_time)
            
            self.send_esp32_command("STOP")
            self.get_logger().info("✅ Actuators retracted - Shelf released")
            return True
        else:
            self.get_logger().error("❌ Failed to retract actuators")
            return False

    # ============================================================
    # AprilTag Alignment
    # ============================================================

    def alignment_status_callback(self, msg):
        """Callback for alignment status updates"""
        self.alignment_status = msg.data
        
        if self.alignment_status == "ALIGNED" and self.task_state == TaskState.ALIGNING_TAG:
            self.get_logger().info("🎯 AprilTag alignment complete!")
            self.task_state = TaskState.ALIGNED
            self.publish_status("ALIGNED")
            
            # Disable alignment
            self.enable_apriltag_alignment(False)
            
            # Now extend actuators
            if self.extend_actuators():
                self.task_state = TaskState.ATTACHED
                self.publish_status("ATTACHED")
                # Proceed to drop location
                time.sleep(1.0)
                self.move_to_drop()
            else:
                self.task_state = TaskState.ERROR
                self.publish_status("ERROR - Actuator extension failed")

    def enable_apriltag_alignment(self, enable: bool):
        """Enable or disable AprilTag alignment"""
        if not self._alignment_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("AprilTag alignment service not available!")
            return False

        request = SetBool.Request()
        request.data = enable
        
        future = self._alignment_client.call_async(request)
        self.get_logger().info(f"{'Enabling' if enable else 'Disabling'} AprilTag alignment")
        return True

    # ============================================================
    # MQTT lifecycle
    # ============================================================

    def _start_mqtt(self):
        try:
            self.get_logger().info(f"Connecting to MQTT broker {self.mqtt_host}:{self.mqtt_port}...")
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Initial MQTT connect error: {e}")
            self._schedule_reconnect()

    def _schedule_reconnect(self):
        with self._mqtt_lock:
            self._mqtt_reconnect_attempts += 1
            delay = min(self._reconnect_base * (2 ** (self._mqtt_reconnect_attempts - 1)), self._reconnect_max)
            self.get_logger().info(f"Scheduling MQTT reconnect in {delay:.1f}s")
            timer = threading.Timer(delay, self._attempt_reconnect)
            timer.daemon = True
            timer.start()

    def _attempt_reconnect(self):
        with self._mqtt_lock:
            try:
                self.get_logger().info("Attempting MQTT reconnect...")
                self.mqtt_client.reconnect()
            except Exception as e:
                self.get_logger().warn(f"Reconnect failed: {e}")
                self._schedule_reconnect()

    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.get_logger().info("MQTT connected successfully")
            self._mqtt_connected = True
            self._mqtt_reconnect_attempts = 0

            try:
                client.subscribe(f"robot/{self.robot_id}/task/assignment", qos=self.mqtt_qos)
                client.subscribe(f"robot/{self.robot_id}/reference_point/update", qos=self.mqtt_qos)
                client.subscribe("robot/all/task/assignment", qos=self.mqtt_qos)
                self.get_logger().info("MQTT subscriptions set")
            except Exception as e:
                self.get_logger().error(f"Subscription failed: {e}")
        else:
            self.get_logger().error(f"MQTT failed to connect, rc={rc}")
            self._schedule_reconnect()

    def _on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warn(f"Unexpected MQTT disconnect (rc={rc})")
            self._schedule_reconnect()
        else:
            self.get_logger().info("MQTT cleanly disconnected")

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode("utf-8")
            data = json.loads(payload)
        except Exception as e:
            self.get_logger().error(f"Failed to decode MQTT payload: {e}")
            return

        topic = msg.topic
        if topic.endswith("/task/assignment") or "task/assignment" in topic:
            self.get_logger().info(f"MQTT Task assignment received")
            if isinstance(data, dict):
                self.on_task_assignment(data)
        elif topic.endswith("/reference_point/update") or "reference_point/update" in topic:
            self.get_logger().info("MQTT Reference point update received")
            if isinstance(data, dict):
                self.on_reference_point_update(data)

    # ============================================================
    # Task handling
    # ============================================================

    def on_task_assignment(self, task_data: Dict[str, Any]):
        if self.task_state != TaskState.IDLE:
            self.get_logger().warn(f"Robot busy ({self.task_state.value}), ignoring task")
            self._publish_mqtt({
                "task_id": task_data.get("task_id"),
                "robot_id": self.robot_id,
                "status": "REJECTED_BUSY",
                "reason": f"Robot in state {self.task_state.value}",
                "timestamp": time.time()
            })
            return

        required = ["task_id", "pickup_x", "pickup_y", "drop_x", "drop_y"]
        for f in required:
            if f not in task_data:
                self.get_logger().error(f"Task missing field: {f}")
                self._publish_mqtt({
                    "task_id": task_data.get("task_id"),
                    "robot_id": self.robot_id,
                    "status": "ERROR",
                    "reason": f"Missing field: {f}",
                    "timestamp": time.time()
                })
                return

        self.current_task = task_data
        self.task_state = TaskState.ASSIGNED

        # Set reference point from task
        try:
            if task_data.get("drop_x") is not None:
                self.on_reference_point_update({
                    "x": float(task_data.get("drop_x", 0.0)),
                    "y": float(task_data.get("drop_y", 0.0)),
                    "yaw": float(task_data.get("drop_yaw", 0.0))
                })
        except Exception as e:
            self.get_logger().warn(f"Failed to set reference point: {e}")

        # Log task details including path info
        path_to_pickup = task_data.get("path_to_pickup", {})
        num_waypoints = path_to_pickup.get("num_waypoints", 0)
        path_distance = path_to_pickup.get("distance", 0)
        
        self.get_logger().info(
            f"📋 Task {task_data.get('task_id')} | "
            f"Pickup: ({task_data.get('pickup_x')},{task_data.get('pickup_y')}) | "
            f"Drop: ({task_data.get('drop_x')},{task_data.get('drop_y')}) | "
            f"Path: {num_waypoints} waypoints, {path_distance:.2f}m"
        )

        self.publish_status("ASSIGNED")
        self.move_to_pickup()

    def on_reference_point_update(self, point_data: Dict[str, float]):
        try:
            self.reference_point = {
                "x": float(point_data.get("x", 0.0)),
                "y": float(point_data.get("y", 0.0)),
                "yaw": float(point_data.get("yaw", 0.0)),
            }
            self.get_logger().info(f"Reference point: {self.reference_point}")
        except Exception as e:
            self.get_logger().error(f"Invalid reference point: {e}")

    # ============================================================
    # Path Creation Helper
    # ============================================================

    def _create_path_msg(self, waypoints: List[Tuple[float, float]], yaw: float = 0.0) -> Path:
        """
        Create a Nav2 Path message from waypoints.
        
        Args:
            waypoints: List of (x, y) tuples in world coordinates
            yaw: Final orientation at goal
        
        Returns:
            Path message for Nav2
        """
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i, (x, y) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            
            # Calculate orientation towards next waypoint (except last)
            if i < len(waypoints) - 1:
                next_x, next_y = waypoints[i + 1]
                dx = next_x - x
                dy = next_y - y
                waypoint_yaw = math.atan2(dy, dx)
            else:
                # Last waypoint uses provided yaw
                waypoint_yaw = yaw
            
            qz = math.sin(waypoint_yaw / 2.0)
            qw = math.cos(waypoint_yaw / 2.0)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            path.poses.append(pose)
        
        return path

    # ============================================================
    # Navigation functions
    # ============================================================

    def move_to_pickup(self):
        if not self.current_task:
            return

        self.task_state = TaskState.MOVING_TO_PICKUP
        self.publish_status("MOVING_TO_PICKUP")

        pickup_x = float(self.current_task.get("pickup_x", 0.0))
        pickup_y = float(self.current_task.get("pickup_y", 0.0))
        pickup_yaw = float(self.current_task.get("pickup_yaw", 0.0))

        # Get path from task assignment
        path_data = self.current_task.get("path_to_pickup")
        
        if self.use_path_following and path_data and path_data.get("waypoints"):
            # Use FollowPath with provided waypoints
            waypoints = path_data["waypoints"]
            self.get_logger().info(
                f"➡️  Following path to pickup: {len(waypoints)} waypoints, "
                f"{path_data.get('distance', 0):.2f}m"
            )
            
            # Create path message
            path_msg = self._create_path_msg(waypoints, pickup_yaw)
            
            # Send to FollowPath action
            goal = FollowPath.Goal()
            goal.path = path_msg
            
            if not self._path_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn("FollowPath not ready, falling back to NavigateToPose...")
                self._navigate_to_pose(pickup_x, pickup_y, pickup_yaw, TaskState.ARRIVED_AT_PICKUP)
            else:
                fut = self._path_client.send_goal_async(goal)
                fut.add_done_callback(lambda f: self._handle_nav_result(f, TaskState.ARRIVED_AT_PICKUP))
        else:
            # Fallback to simple NavigateToPose
            self.get_logger().info(f"➡️  Moving to pickup: ({pickup_x:.2f}, {pickup_y:.2f})")
            self._navigate_to_pose(pickup_x, pickup_y, pickup_yaw, TaskState.ARRIVED_AT_PICKUP)

    def move_to_drop(self):
        if not self.current_task:
            return

        self.task_state = TaskState.MOVING_TO_DROP
        self.publish_status("MOVING_TO_DROP")

        drop_x = float(self.current_task.get("drop_x", 0.0))
        drop_y = float(self.current_task.get("drop_y", 0.0))
        drop_yaw = float(self.current_task.get("drop_yaw", 0.0))

        # Get path from task assignment
        path_data = self.current_task.get("path_to_drop")
        
        if self.use_path_following and path_data and path_data.get("waypoints"):
            # Use FollowPath with provided waypoints
            waypoints = path_data["waypoints"]
            self.get_logger().info(
                f"➡️  Following path to drop: {len(waypoints)} waypoints, "
                f"{path_data.get('distance', 0):.2f}m"
            )
            
            # Create path message
            path_msg = self._create_path_msg(waypoints, drop_yaw)
            
            # Send to FollowPath action
            goal = FollowPath.Goal()
            goal.path = path_msg
            
            if not self._path_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn("FollowPath not ready, falling back to NavigateToPose...")
                self._navigate_to_pose(drop_x, drop_y, drop_yaw, TaskState.ARRIVED_AT_DROP)
            else:
                fut = self._path_client.send_goal_async(goal)
                fut.add_done_callback(lambda f: self._handle_nav_result(f, TaskState.ARRIVED_AT_DROP))
        else:
            # Fallback to simple NavigateToPose
            self.get_logger().info(f"➡️  Moving to drop: ({drop_x:.2f}, {drop_y:.2f})")
            self._navigate_to_pose(drop_x, drop_y, drop_yaw, TaskState.ARRIVED_AT_DROP)

    def move_to_reference(self):
        if not self.reference_point:
            self.get_logger().warn("Reference point not set")
            self.task_state = TaskState.ERROR
            self.publish_status("ERROR")
            return

        self.task_state = TaskState.MOVING_TO_REFERENCE
        self.publish_status("MOVING_TO_REFERENCE")

        x = self.reference_point["x"]
        y = self.reference_point["y"]
        yaw = self.reference_point["yaw"]

        self.get_logger().info("🏠 Returning to reference point")
        self._navigate_to_pose(x, y, yaw, TaskState.COMPLETED)

    def _navigate_to_pose(self, x: float, y: float, yaw: float, next_state: TaskState):
        """Helper to use NavigateToPose action"""
        goal = self._create_nav_goal(x, y, yaw)
        
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 not ready, trying anyway...")
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._handle_nav_result(f, next_state))

    # ============================================================
    # Navigation callbacks
    # ============================================================

    def _handle_nav_result(self, future, next_state: TaskState):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to get goal handle: {e}")
            self.task_state = TaskState.ERROR
            self.publish_status("ERROR")
            return

        if not handle.accepted:
            self.get_logger().error("Nav2 rejected goal")
            self.task_state = TaskState.ERROR
            self.publish_status("ERROR")
            return

        self.get_logger().info("Nav2 accepted goal...")
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(lambda f: self._handle_nav_complete(f, next_state))

    def _handle_nav_complete(self, future, next_state: TaskState):
        try:
            res = future.result()
            self.get_logger().info(f"✅ Reached destination ({next_state.value})")

            # Update position
            if self.current_task:
                if next_state == TaskState.ARRIVED_AT_PICKUP:
                    self.update_robot_position(
                        float(self.current_task.get("pickup_x", 0.0)),
                        float(self.current_task.get("pickup_y", 0.0)),
                        float(self.current_task.get("pickup_yaw", 0.0))
                    )
                elif next_state == TaskState.ARRIVED_AT_DROP:
                    self.update_robot_position(
                        float(self.current_task.get("drop_x", 0.0)),
                        float(self.current_task.get("drop_y", 0.0)),
                        float(self.current_task.get("drop_yaw", 0.0))
                    )
                elif next_state == TaskState.COMPLETED and self.reference_point:
                    self.update_robot_position(
                        self.reference_point["x"],
                        self.reference_point["y"],
                        self.reference_point["yaw"]
                    )

            # Handle different states
            if next_state == TaskState.ARRIVED_AT_PICKUP:
                self.task_state = TaskState.ARRIVED_AT_PICKUP
                self.publish_status("ARRIVED_AT_PICKUP")
                self.get_logger().info("🎯 Starting AprilTag alignment...")
                
                # Enable AprilTag alignment
                time.sleep(1.0)
                self.task_state = TaskState.ALIGNING_TAG
                self.publish_status("ALIGNING_TAG")
                self.enable_apriltag_alignment(True)

            elif next_state == TaskState.ARRIVED_AT_DROP:
                self.task_state = TaskState.ARRIVED_AT_DROP
                self.publish_status("ARRIVED_AT_DROP")
                self.get_logger().info("📦 Arrived at drop location")
                
                # Retract actuators to release shelf
                time.sleep(1.0)
                if self.retract_actuators():
                    self.task_state = TaskState.RELEASED
                    self.publish_status("RELEASED")
                    self.get_logger().info("✅ Shelf released")
                    time.sleep(1.0)
                    self.move_to_reference()
                else:
                    self.task_state = TaskState.ERROR
                    self.publish_status("ERROR - Actuator retraction failed")

            elif next_state == TaskState.COMPLETED:
                self.task_state = TaskState.COMPLETED
                self.publish_status("COMPLETED")
                self.get_logger().info("=" * 60)
                self.get_logger().info("🎉 TASK COMPLETED SUCCESSFULLY")
                self.get_logger().info("=" * 60)
                self.current_task = None
                self.task_state = TaskState.IDLE

        except Exception as e:
            self.get_logger().error(f"Error in nav completion: {e}")
            self.task_state = TaskState.ERROR
            self.publish_status("ERROR")

    # ============================================================
    # Helpers
    # ============================================================

    def _create_nav_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qz = math.sin(float(yaw) / 2.0)
        qw = math.cos(float(yaw) / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose
        return goal

    def publish_status(self, status: str):
        msg_data = {
            "task_id": self.current_task.get("task_id") if self.current_task else None,
            "robot_id": self.robot_id,
            "status": status,
            "task_state": self.task_state.value,
            "timestamp": time.time(),
        }

        if self.current_task:
            msg_data.update({
                "shelf_id": self.current_task.get("shelf_id"),
                "priority": self.current_task.get("priority"),
            })

        # ROS2
        ros_msg = String()
        ros_msg.data = json.dumps(msg_data)
        try:
            self.task_status_pub.publish(ros_msg)
        except Exception as e:
            self.get_logger().debug(f"Failed to publish ROS2 status: {e}")

        # MQTT
        self._publish_mqtt(msg_data)
        
        try:
            self._publish_mqtt({
                "task_id": msg_data.get("task_id"),
                "robot_id": self.robot_id,
                "task_state": self.task_state.value,
                "status": status,
                "timestamp": time.time()
            }, topic_suffix="task/progress")
        except Exception as e:
            self.get_logger().debug(f"Failed to publish progress: {e}")

    def _publish_mqtt(self, payload: Dict[str, Any], topic_suffix: str = "task/status"):
        try:
            topic = f"robot/{self.robot_id}/{topic_suffix}"
            payload_str = json.dumps(payload)
            self.mqtt_client.publish(topic, payload_str, qos=self.mqtt_qos)
        except Exception as e:
            self.get_logger().error(f"MQTT publish failed: {e}")

    def update_robot_position(self, x: float, y: float, yaw: float = 0.0):
        with self._position_lock:
            self.robot_x = float(x)
            self.robot_y = float(y)
            self.robot_yaw = float(yaw)
            self.last_position_update = time.time()
        
        if self.backend_enabled and self.current_task:
            self._send_position_to_backend(x, y, yaw)
        
        try:
            self._publish_mqtt({
                "robot_id": self.robot_id,
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
                "timestamp": time.time()
            }, topic_suffix="position/update")
        except Exception as e:
            self.get_logger().debug(f"Failed to publish position: {e}")
    
    def _send_position_to_backend(self, x: float, y: float, yaw: float):
        if not self.current_task:
            return
        
        task_id = self.current_task.get("task_id")
        if not task_id:
            return
        
        try:
            url = f"http://{self.backend_host}:{self.backend_port}/api/tasks/realtime/{task_id}/position"
            payload = {
                "robot_id": self.robot_id,
                "current_x": float(x),
                "current_y": float(y),
                "current_theta": float(yaw),
                "status": self.task_state.value,
                "timestamp": time.time()
            }
            requests.post(url, json=payload, timeout=2.0)
        except:
            pass

    # ============================================================
    # Shutdown
    # ============================================================

    def destroy_node(self):
        try:
            # Stop ESP32
            if self.esp32_serial and self.esp32_serial.is_open:
                self.send_esp32_command("STOP")
                self.esp32_serial.close()
                self.get_logger().info("ESP32 connection closed")
            
            # Disable alignment
            self.enable_apriltag_alignment(False)
            
            # Stop MQTT
            if hasattr(self, "mqtt_client"):
                try:
                    self.mqtt_client.disconnect()
                    self.mqtt_client.loop_stop()
                except:
                    pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = RobotTaskRunner()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()