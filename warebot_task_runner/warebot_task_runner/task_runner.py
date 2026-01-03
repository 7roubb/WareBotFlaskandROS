#!/usr/bin/env python3
"""
WareBot Task Runner with Integrated AprilTag Alignment
✅ ROBUST MQTT STATUS UPDATES
✅ TASK QUEUE MANAGEMENT
✅ RELIABLE MESSAGE DELIVERY

Features:
- Persistent task queue per robot
- Multiple status publish attempts with retries
- Confirmation of message delivery
- Proper state machine transitions
"""

import json
import math
import time
import threading
import os
from enum import Enum
from typing import Optional, Dict, Any
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

from cv_bridge import CvBridge
import cv2
import numpy as np
import paho.mqtt.client as mqtt
from pupil_apriltags import Detector


class TaskState(Enum):
    """Task execution states"""
    IDLE = "IDLE"
    ASSIGNED = "ASSIGNED"
    MOVING_TO_PICKUP = "MOVING_TO_PICKUP"
    ARRIVED_AT_PICKUP = "ARRIVED_AT_PICKUP"
    ALIGNING = "ALIGNING"
    ALIGNED = "ALIGNED"
    MOVING_TO_DROP = "MOVING_TO_DROP"
    ARRIVED_AT_DROP = "ARRIVED_AT_DROP"
    RELEASED = "RELEASED"
    MOVING_TO_REFERENCE = "MOVING_TO_REFERENCE"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"


class PID:
    """PID Controller"""
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, i_limit=1.0, out_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = abs(i_limit)
        self.out_limit = abs(out_limit)
        self.i = 0.0
        self.prev_e = None
        self.prev_t = None

    def reset(self):
        self.i = 0.0
        self.prev_e = None
        self.prev_t = None

    def step(self, e: float) -> float:
        t = time.time()
        dt = 0.0 if self.prev_t is None else max(1e-3, t - self.prev_t)
        p = self.kp * e
        self.i += e * dt
        self.i = float(np.clip(self.i, -self.i_limit, self.i_limit))
        d = 0.0 if self.prev_e is None else self.kd * (e - self.prev_e) / dt
        self.prev_e = e
        self.prev_t = t
        u = p + self.ki * self.i + d
        return float(np.clip(u, -self.out_limit, self.out_limit))


def apply_min(cmd, min_mag):
    """Apply minimum magnitude to command"""
    if abs(cmd) < 1e-6:
        return 0.0
    return math.copysign(max(min_mag, abs(cmd)), cmd)


class RobotTaskRunner(Node):
    """Integrated Task Runner with AprilTag Alignment + Robust MQTT"""

    def __init__(self):
        super().__init__("task_runner")

        # --- ROS2 parameters ---
        self.declare_parameter("robot_id", "robot_1")
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1884)
        self.declare_parameter("mqtt_qos", 1)
        self.declare_parameter("mqtt_reconnect_base", 1.0)
        self.declare_parameter("mqtt_reconnect_max", 30.0)
        
        # AprilTag parameters
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("estop_topic", "/emergency_stop_state")
        self.declare_parameter("show_debug_image", True)
        self.declare_parameter("lost_tag_timeout", 0.7)
        self.declare_parameter("camera_matrix_path", "")
        self.declare_parameter("dist_coeffs_path", "")
        self.declare_parameter("alignment_hold_time", 30.0)

        self.robot_id = self.get_parameter("robot_id").value
        self.mqtt_host = self.get_parameter("mqtt_host").value
        self.mqtt_port = int(self.get_parameter("mqtt_port").value)
        self.mqtt_qos = int(self.get_parameter("mqtt_qos").value)
        self._reconnect_base = float(self.get_parameter("mqtt_reconnect_base").value)
        self._reconnect_max = float(self.get_parameter("mqtt_reconnect_max").value)
        
        # AprilTag parameters
        img_topic = self.get_parameter("image_topic").value
        cmd_topic = self.get_parameter("cmd_vel_topic").value
        estop_topic = self.get_parameter("estop_topic").value
        self.show_debug = self.get_parameter("show_debug_image").value
        self.lost_tag_timeout = self.get_parameter("lost_tag_timeout").value
        self.alignment_hold_time = self.get_parameter("alignment_hold_time").value
        
        cam_matrix_path = self.get_parameter("camera_matrix_path").value
        dist_coeffs_path = self.get_parameter("dist_coeffs_path").value

        self.get_logger().info(f"Task Runner initialized for {self.robot_id}")

        # --- State ---
        self.current_task: Optional[Dict[str, Any]] = None
        self.task_state = TaskState.IDLE
        self.reference_point: Optional[Dict[str, float]] = None
        
        # ✅ TASK QUEUE - Store pending tasks
        self.task_queue = deque()
        self.queue_lock = threading.Lock()

        # ✅ STATUS PUBLISHING QUEUE - Robust status updates
        self.status_update_queue = deque()
        self.status_publish_lock = threading.Lock()
        self.last_status = None
        self.status_retry_count = {}

        # --- Nav2 action client ---
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for Nav2 server...")
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("Nav2 server not available yet. Still continuing.")
        else:
            self.get_logger().info("Nav2 server ready")

        # --- ROS2 publishers ---
        self.task_status_pub = self.create_publisher(String, f"/{self.robot_id}/task/status", 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # --- Robot position tracking ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_position_update = time.time()
        self._position_lock = threading.Lock()

        # --- AprilTag initialization ---
        if not os.path.exists(cam_matrix_path):
            self.get_logger().error(f"❌ Camera matrix not found: {cam_matrix_path}")
            raise FileNotFoundError(f"Camera matrix not found: {cam_matrix_path}")
        if not os.path.exists(dist_coeffs_path):
            self.get_logger().error(f"❌ Distortion coefficients not found: {dist_coeffs_path}")
            raise FileNotFoundError(f"Distortion coefficients not found: {dist_coeffs_path}")

        try:
            self.camera_matrix = np.load(cam_matrix_path)
            self.dist_coeffs = np.load(dist_coeffs_path)
            self.get_logger().info(f"✅ Loaded camera matrix")
            self.get_logger().info(f"✅ Loaded distortion coefficients")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load calibration files: {e}")
            raise

        self.bridge = CvBridge()
        self.detector = Detector()
        
        # AprilTag parameters
        self.TAG_SIZE = 0.16
        self.TOLERANCE_X = 0.09
        self.TOLERANCE_Z = 0.09
        self.TOLERANCE_YAW = 4.0

        self.obj_pts = np.array([
            [-0.08,  0.08, 0],
            [ 0.08,  0.08, 0],
            [ 0.08, -0.08, 0],
            [-0.08, -0.08, 0],
        ], dtype=np.float32)

        # PID Controllers
        self.pid_yaw     = PID(0.06, 0.0, 0.020, 0.5, 0.35)
        self.pid_strafe  = PID(1.2, 0.0, 0.25, 0.4, 0.15)
        self.pid_forward = PID(1.2, 0.0, 0.25, 0.4, 0.10)

        self.min_wz = 0.05
        self.min_vy = 0.02
        self.min_vx = 0.02

        # AprilTag state
        self.sx = self.sz = self.syaw = None
        self.x = self.z = self.yaw_deg = 0.0
        self.tag_found = False
        self.last_tag_time = None
        self.estop_active = False
        self.aligned = False
        self.alignment_time = None
        self.alignment_complete = False
        self.should_exit = False
        self.frame_count = 0
        self.no_tag_count = 0

        # ROS2 subscriptions
        self.img_sub = self.create_subscription(Image, img_topic, self.image_cb, 10)
        self.estop_sub = self.create_subscription(Bool, estop_topic, self.estop_cb, 10)

        # Control loop timer (when aligning)
        self.control_timer = None
        
        # ✅ Status publisher timer - ensures continuous delivery
        self.status_timer = self.create_timer(0.5, self.process_status_queue)

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
        self.mqtt_client.on_publish = self._on_mqtt_publish

        self._mqtt_connected = False
        self._mqtt_reconnect_attempts = 0
        self._mqtt_lock = threading.Lock()

        self._start_mqtt()
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("✅ Task Runner with Robust MQTT Started")
        self.get_logger().info(f"📷 Image topic: {img_topic}")
        self.get_logger().info(f"🎮 Cmd vel topic: {cmd_topic}")
        self.get_logger().info(f"📡 MQTT: {self.mqtt_host}:{self.mqtt_port}")
        self.get_logger().info("=" * 50)

    # ================= MQTT LIFECYCLE =================

    def _start_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.get_logger().info(f"Connecting to MQTT broker {self.mqtt_host}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT connect error: {e}")
            self._schedule_reconnect()

    def _schedule_reconnect(self):
        """Schedule reconnect with exponential backoff"""
        with self._mqtt_lock:
            self._mqtt_reconnect_attempts += 1
            delay = min(
                self._reconnect_base * (2 ** (self._mqtt_reconnect_attempts - 1)),
                self._reconnect_max
            )
            self.get_logger().info(f"Scheduling MQTT reconnect in {delay:.1f}s")
            timer = threading.Timer(delay, self._attempt_reconnect)
            timer.daemon = True
            timer.start()

    def _attempt_reconnect(self):
        """Try to reconnect"""
        with self._mqtt_lock:
            try:
                self.get_logger().info("Attempting MQTT reconnect...")
                self.mqtt_client.reconnect()
            except Exception as e:
                self.get_logger().warn(f"Reconnect failed: {e}")
                self._schedule_reconnect()

    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT connect callback"""
        if rc == 0:
            self.get_logger().info("✅ MQTT connected")
            self._mqtt_connected = True
            self._mqtt_reconnect_attempts = 0

            try:
                client.subscribe(f"robot/{self.robot_id}/task/assignment", qos=self.mqtt_qos)
                client.subscribe(f"robot/{self.robot_id}/reference_point/update", qos=self.mqtt_qos)
                client.subscribe("robot/all/task/assignment", qos=self.mqtt_qos)
                self.get_logger().info("✅ MQTT subscriptions set")
                
                # ✅ Publish ready status when connected
                self._queue_status_update("READY")
            except Exception as e:
                self.get_logger().error(f"Subscription failed: {e}")
        else:
            self.get_logger().error(f"❌ MQTT connection failed, rc={rc}")
            self._schedule_reconnect()

    def _on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        """MQTT disconnect callback"""
        self._mqtt_connected = False
        if rc != 0:
            self.get_logger().warn(f"❌ Unexpected MQTT disconnect (rc={rc})")
            self._schedule_reconnect()

    def _on_mqtt_publish(self, client, userdata, mid):
        """MQTT publish callback - confirms delivery"""
        self.get_logger().debug(f"📤 MQTT message {mid} published successfully")

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            payload = msg.payload.decode("utf-8")
            data = json.loads(payload)
        except Exception as e:
            self.get_logger().error(f"Failed to decode MQTT payload: {e}")
            return

        topic = msg.topic
        if "task/assignment" in topic:
            self.get_logger().info(f"📥 Task assignment received via MQTT: {data.get('task_id')}")
            if isinstance(data, dict):
                self.on_task_assignment(data)
        elif "reference_point/update" in topic:
            self.get_logger().info("Reference point update received")
            if isinstance(data, dict):
                self.on_reference_point_update(data)

    # ================= ROBUST STATUS PUBLISHING =================

    def _queue_status_update(self, status: str):
        """Queue a status update for robust delivery with retries"""
        with self.status_publish_lock:
            self.status_update_queue.append({
                "status": status,
                "task_id": self.current_task.get("task_id") if self.current_task else None,
                "timestamp": time.time(),
                "retry_count": 0
            })
            self.get_logger().info(f"📋 Status queued: {status}")

    def process_status_queue(self):
        """Process status update queue with retries"""
        with self.status_publish_lock:
            if not self.status_update_queue:
                return
            
            status_item = self.status_update_queue[0]
        
        try:
            if not self._mqtt_connected:
                self.get_logger().debug("⏳ MQTT not connected, retrying status publish...")
                status_item["retry_count"] += 1
                if status_item["retry_count"] > 10:
                    # Give up after 10 retries (5 seconds)
                    with self.status_publish_lock:
                        self.status_update_queue.popleft()
                    self.get_logger().warn(f"❌ Gave up publishing status after 10 retries")
                return
            
            # Build status message
            msg_data = {
                "task_id": status_item.get("task_id"),
                "robot_id": self.robot_id,
                "status": status_item["status"],
                "task_state": self.task_state.value,
                "timestamp": status_item["timestamp"],
            }

            if self.current_task:
                msg_data.update({
                    "shelf_id": self.current_task.get("shelf_id"),
                    "pickup_location": {
                        "x": float(self.current_task.get("pickup_x", 0.0)),
                        "y": float(self.current_task.get("pickup_y", 0.0))
                    },
                    "drop_location": {
                        "x": float(self.current_task.get("drop_x", 0.0)),
                        "y": float(self.current_task.get("drop_y", 0.0))
                    }
                })

            # Publish to ROS2
            ros_msg = String()
            ros_msg.data = json.dumps(msg_data)
            try:
                self.task_status_pub.publish(ros_msg)
            except Exception as e:
                self.get_logger().debug(f"ROS2 publish failed: {e}")

            # ✅ CRITICAL: Publish to MQTT with guaranteed delivery
            self._publish_mqtt_with_retry(msg_data, status_item)
            
            # Remove from queue after successful publish
            with self.status_publish_lock:
                if self.status_update_queue and self.status_update_queue[0] == status_item:
                    self.status_update_queue.popleft()
            
            self.get_logger().info(f"✅ Status published: {status_item['status']} (attempt {status_item['retry_count'] + 1})")

        except Exception as e:
            self.get_logger().error(f"Error processing status queue: {e}")
            status_item["retry_count"] += 1
            if status_item["retry_count"] > 10:
                with self.status_publish_lock:
                    if self.status_update_queue and self.status_update_queue[0] == status_item:
                        self.status_update_queue.popleft()

    def _publish_mqtt_with_retry(self, payload: Dict[str, Any], status_item: Dict):
        """Publish to MQTT with automatic retry on failure"""
        try:
            topic = f"robot/{self.robot_id}/task/status"
            payload_str = json.dumps(payload)
            
            # Publish with QoS=1 (at least once delivery)
            result = self.mqtt_client.publish(
                topic,
                payload_str,
                qos=self.mqtt_qos,
                retain=False
            )
            
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                status_item["retry_count"] += 1
                self.get_logger().warn(
                    f"⚠️  MQTT publish returned rc={result.rc}, will retry "
                    f"(attempt {status_item['retry_count']}/10)"
                )
            else:
                self.get_logger().debug(f"📤 Published to {topic}")
                
        except Exception as e:
            status_item["retry_count"] += 1
            self.get_logger().error(f"MQTT publish error: {e} (attempt {status_item['retry_count']}/10)")

    # ================= TASK QUEUE MANAGEMENT =================

    def _queue_task(self, task_data: Dict[str, Any]):
        """Add task to queue for processing"""
        with self.queue_lock:
            self.task_queue.append(task_data)
            self.get_logger().info(f"📋 Task queued: {task_data.get('task_id')} (queue size: {len(self.task_queue)})")

    def _process_next_task(self):
        """Process next task from queue if robot is IDLE"""
        if self.task_state != TaskState.IDLE:
            return
        
        with self.queue_lock:
            if not self.task_queue:
                self.get_logger().debug("✓ Task queue empty")
                return
            
            next_task = self.task_queue.popleft()
        
        self.get_logger().info(f"▶️  Processing task from queue: {next_task.get('task_id')}")
        self.on_task_assignment(next_task)

    # ================= TASK HANDLING =================

    def on_task_assignment(self, task_data: Dict[str, Any]):
        """Handle task assignment"""
        if self.task_state != TaskState.IDLE:
            self.get_logger().warn(f"🚫 Robot busy ({self.task_state.value}), queuing task")
            # Queue for later processing
            self._queue_task(task_data)
            
            self._publish_mqtt({
                "task_id": task_data.get("task_id"),
                "robot_id": self.robot_id,
                "status": "QUEUED",
                "reason": f"Robot in state {self.task_state.value}",
                "timestamp": time.time()
            }, "task/status")
            return

        # Validate required fields
        required = ["task_id", "pickup_x", "pickup_y", "drop_x", "drop_y"]
        for f in required:
            if f not in task_data:
                self.get_logger().error(f"❌ Task missing field: {f}")
                self._publish_mqtt({
                    "task_id": task_data.get("task_id"),
                    "robot_id": self.robot_id,
                    "status": "ERROR",
                    "reason": f"Missing field: {f}",
                    "timestamp": time.time()
                }, "task/status")
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

        self.get_logger().info(
            f"✅ Task assigned: {task_data.get('task_id')} | "
            f"Pickup: ({task_data.get('pickup_x')},{task_data.get('pickup_y')}) | "
            f"Drop: ({task_data.get('drop_x')},{task_data.get('drop_y')})"
        )

        # ✅ Queue status update instead of direct publish
        self._queue_status_update("ASSIGNED")
        self.move_to_pickup()

    def on_reference_point_update(self, point_data: Dict[str, float]):
        """Update reference point"""
        try:
            self.reference_point = {
                "x": float(point_data.get("x", 0.0)),
                "y": float(point_data.get("y", 0.0)),
                "yaw": float(point_data.get("yaw", 0.0)),
            }
            self.get_logger().info(f"Reference point: {self.reference_point}")
        except Exception as e:
            self.get_logger().error(f"Invalid reference point: {e}")

    # ================= NAVIGATION =================

    def move_to_pickup(self):
        """Navigate to pickup"""
        if not self.current_task:
            self.get_logger().warn("No current task")
            return

        self.task_state = TaskState.MOVING_TO_PICKUP
        self._queue_status_update("MOVING_TO_PICKUP")

        x = float(self.current_task.get("pickup_x", 0.0))
        y = float(self.current_task.get("pickup_y", 0.0))
        yaw = float(self.current_task.get("pickup_yaw", 0.0))

        self.get_logger().info(f"🗺️  Moving to pickup: ({x}, {y})")
        goal = self._create_nav_goal(x, y, yaw)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 not ready")
        
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._handle_pickup_arrival(f))

    def move_to_drop(self):
        """Navigate to drop location"""
        if not self.current_task:
            self.get_logger().warn("No current task")
            return

        self.task_state = TaskState.MOVING_TO_DROP
        self._queue_status_update("MOVING_TO_DROP")

        x = float(self.current_task.get("drop_x", 0.0))
        y = float(self.current_task.get("drop_y", 0.0))
        yaw = float(self.current_task.get("drop_yaw", 0.0))

        self.get_logger().info(f"🗺️  Moving to drop: ({x}, {y})")
        goal = self._create_nav_goal(x, y, yaw)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 not ready")
        
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._handle_drop_arrival(f))

    def move_to_reference(self):
        """Return to reference point"""
        if not self.reference_point:
            self.get_logger().warn("Reference point not set")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        self.task_state = TaskState.MOVING_TO_REFERENCE
        self._queue_status_update("MOVING_TO_REFERENCE")

        x = self.reference_point["x"]
        y = self.reference_point["y"]
        yaw = self.reference_point["yaw"]

        self.get_logger().info("🗺️  Returning to reference point")
        goal = self._create_nav_goal(x, y, yaw)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 not ready")
        
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._handle_reference_arrival(f))

    # ================= NAVIGATION CALLBACKS =================

    def _handle_pickup_arrival(self, future):
        """Handle arrival at pickup location"""
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal failed: {e}")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        if not handle.accepted:
            self.get_logger().error("Nav2 rejected goal")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        self.get_logger().info("Nav2 goal accepted, waiting...")
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._on_pickup_reached)

    def _on_pickup_reached(self, future):
        """Reached pickup location - start alignment"""
        try:
            res = future.result()
            self.get_logger().info("Reached pickup location")
            self.update_robot_position(
                float(self.current_task.get("pickup_x", 0.0)),
                float(self.current_task.get("pickup_y", 0.0)),
                float(self.current_task.get("pickup_yaw", 0.0))
            )
        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        # Start alignment
        self.on_arrived_pickup()

    def _handle_drop_arrival(self, future):
        """Handle arrival at drop location"""
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal failed: {e}")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        if not handle.accepted:
            self.get_logger().error("Nav2 rejected goal")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._on_drop_reached)

    def _on_drop_reached(self, future):
        """Reached drop location"""
        try:
            res = future.result()
            self.get_logger().info("Reached drop location")
            self.update_robot_position(
                float(self.current_task.get("drop_x", 0.0)),
                float(self.current_task.get("drop_y", 0.0)),
                float(self.current_task.get("drop_yaw", 0.0))
            )
        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        self.task_state = TaskState.RELEASED
        self._queue_status_update("RELEASED")
        self.get_logger().info("Shelf released")
        time.sleep(1.0)
        self.move_to_reference()

    def _handle_reference_arrival(self, future):
        """Handle arrival at reference point"""
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal failed: {e}")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        if not handle.accepted:
            self.get_logger().error("Nav2 rejected goal")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._on_reference_reached)

    def _on_reference_reached(self, future):
        """Reached reference point - task complete"""
        try:
            res = future.result()
            self.get_logger().info("Returned to reference point")
            if self.reference_point:
                self.update_robot_position(
                    self.reference_point["x"],
                    self.reference_point["y"],
                    self.reference_point["yaw"]
                )
        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            self.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        self.task_state = TaskState.COMPLETED
        self._queue_status_update("COMPLETED")
        self.get_logger().info("✅ Task completed successfully")
        self.current_task = None
        self.task_state = TaskState.IDLE
        
        # ✅ Process next task from queue
        self._process_next_task()

    # ================= ALIGNMENT CONTROL =================

    def on_arrived_pickup(self):
        """Start AprilTag alignment"""
        self.task_state = TaskState.ALIGNING
        self._queue_status_update("ALIGNING")
        self.get_logger().info("🚀 Starting AprilTag alignment")
        
        # Reset alignment state
        self.alignment_complete = False
        self.should_exit = False
        self.aligned = False
        self.alignment_time = None
        self.tag_found = False
        self.frame_count = 0
        self.no_tag_count = 0
        
        # Start control loop timer
        self.control_timer = self.create_timer(0.05, self.alignment_control_loop)

    # ================= APRILTAG CALLBACKS =================

    def estop_cb(self, msg):
        """Emergency stop callback"""
        self.estop_active = msg.data
        if msg.data:
            self.get_logger().warn("⚠️  EMERGENCY STOP ACTIVATED")
            self.pid_yaw.reset()
            self.pid_strafe.reset()
            self.pid_forward.reset()
        else:
            self.get_logger().info("✅ Emergency stop deactivated")

    def image_cb(self, msg):
        """Image callback - only process when aligning"""
        if self.task_state != TaskState.ALIGNING or self.should_exit:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ CV Bridge failed: {e}")
            return

        self.frame_count += 1
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        if len(detections) == 0:
            self.tag_found = False
            self.no_tag_count += 1
            
            if self.frame_count % 30 == 0:
                self.get_logger().debug(f"⏳ Waiting for tag... (frame {self.frame_count})")
            return

        # Tag detected
        self.no_tag_count = 0
        det = detections[0]
        corners = np.array(det.corners, dtype=np.float32)

        ok, rvec, tvec = cv2.solvePnP(self.obj_pts, corners, self.camera_matrix, self.dist_coeffs)
        if not ok:
            self.get_logger().warn("❌ solvePnP failed")
            self.tag_found = False
            return

        x = float(tvec[0])
        z = float(tvec[1])

        R, _ = cv2.Rodrigues(rvec)
        yaw = float(math.degrees(math.atan2(-R[1,0], R[0,0])))

        # Low-pass filter
        alpha = 0.3
        self.sx = x if self.sx is None else self.sx*(1-alpha)+x*alpha
        self.sz = z if self.sz is None else self.sz*(1-alpha)+z*alpha
        self.syaw = yaw if self.syaw is None else self.syaw*(1-alpha)+yaw*alpha

        self.x, self.z, self.yaw_deg = self.sx, self.sz, self.syaw
        self.tag_found = True
        self.last_tag_time = time.time()

        # Log alignment status every 10 frames
        if self.frame_count % 10 == 0:
            x_ok = abs(self.x) <= self.TOLERANCE_X
            z_ok = abs(self.z) <= self.TOLERANCE_Z
            yaw_ok = abs(self.yaw_deg) <= self.TOLERANCE_YAW
            
            if x_ok and z_ok and yaw_ok and self.aligned:
                elapsed = time.time() - self.alignment_time
                remaining = max(0, self.alignment_hold_time - elapsed)
                self.get_logger().info(
                    f"✅ ALIGNED | X:{self.x:+.3f} Z:{self.z:+.3f} Yaw:{self.yaw_deg:+.1f}° | Hold: {remaining:.1f}s"
                )

    def alignment_control_loop(self):
        """Control loop for AprilTag alignment"""
        if self.should_exit or self.control_timer is None:
            return

        twist = Twist()

        if self.estop_active:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.cmd_pub.publish(twist)
            return

        if not self.tag_found or (self.last_tag_time and time.time() - self.last_tag_time > self.lost_tag_timeout):
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.cmd_pub.publish(twist)
            return

        # Compute PID
        wz = self.pid_yaw.step(-self.yaw_deg)
        vy = self.pid_strafe.step(-self.x)
        vx = self.pid_forward.step(-self.z)

        # Alignment tolerance checks
        x_ok = abs(self.x) <= self.TOLERANCE_X
        z_ok = abs(self.z) <= self.TOLERANCE_Z
        yaw_ok = abs(self.yaw_deg) <= self.TOLERANCE_YAW

        if x_ok: vy = 0.0
        if z_ok: vx = 0.0
        if yaw_ok: wz = 0.0

        if not x_ok: vy = apply_min(vy, self.min_vy)
        if not z_ok: vx = apply_min(vx, self.min_vx)
        if not yaw_ok: wz = apply_min(wz, self.min_wz)

        twist.angular.z = float(wz)
        twist.linear.y = float(vy)
        twist.linear.x = float(vx)

        self.cmd_pub.publish(twist)

        # ✅ Check for perfect alignment and hold time
        if x_ok and z_ok and yaw_ok:
            if not self.aligned:
                self.aligned = True
                self.alignment_time = time.time()
                self.get_logger().info("🎯 PERFECT ALIGNMENT - Starting hold time")
            else:
                elapsed_time = time.time() - self.alignment_time
                
                if elapsed_time >= self.alignment_hold_time:
                    self.get_logger().info(f"✅ ALIGNMENT COMPLETE - Held for {self.alignment_hold_time}s")
                    
                    # Stop the robot
                    stop_twist = Twist()
                    stop_twist.angular.z = 0.0
                    stop_twist.linear.x = 0.0
                    stop_twist.linear.y = 0.0
                    self.cmd_pub.publish(stop_twist)
                    
                    time.sleep(0.2)
                    
                    if self.control_timer:
                        self.control_timer.cancel()
                        self.control_timer = None
                    
                    self.should_exit = True
                    self.alignment_complete = True
                    
                    self.task_state = TaskState.ALIGNED
                    self._queue_status_update("ALIGNED")
                    
                    self.get_logger().info("🚀 Alignment complete - proceeding to drop location")
                    
                    self.move_to_drop()
                    return
        else:
            if self.aligned:
                self.get_logger().warn("⚠️  Lost alignment - restarting hold time")
                self.aligned = False
                self.alignment_time = None

    # ================= HELPERS =================

    def _create_nav_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        """Create navigation goal"""
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

    def _publish_mqtt(self, payload: Dict[str, Any], topic_suffix: str = "task/status"):
        """Direct MQTT publish (for non-queued messages)"""
        try:
            topic = f"robot/{self.robot_id}/{topic_suffix}"
            payload_str = json.dumps(payload)
            self.mqtt_client.publish(topic, payload_str, qos=self.mqtt_qos)
            self.get_logger().debug(f"📤 Published to {topic}")
        except Exception as e:
            self.get_logger().error(f"MQTT publish failed: {e}")

    def update_robot_position(self, x: float, y: float, yaw: float = 0.0):
        """Update robot position"""
        with self._position_lock:
            self.robot_x = float(x)
            self.robot_y = float(y)
            self.robot_yaw = float(yaw)
            self.last_position_update = time.time()

    def destroy_node(self):
        """Clean shutdown"""
        if self.control_timer:
            self.control_timer.cancel()
        
        if self.status_timer:
            self.status_timer.cancel()

        try:
            twist = Twist()
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.cmd_pub.publish(twist)
        except:
            pass

        try:
            if hasattr(self, "mqtt_client"):
                self.mqtt_client.disconnect()
                self.mqtt_client.loop_stop()
        except Exception:
            pass

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
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()