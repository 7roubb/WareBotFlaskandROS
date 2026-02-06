#!/usr/bin/env python3
"""
Enhanced WareBot Task Runner Node
Fully MQTT-based autonomous navigation for warehouse tasks

- Compatible with paho-mqtt v2.x (uses callback_api_version)
- Basic reconnect/backoff logic on MQTT disconnect
- Publishes status to both ROS2 topic and MQTT
- Uses Nav2 NavigateToPose action for navigation
"""

import json
import math
import time
import threading
from enum import Enum
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Polygon, Point32
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import paho.mqtt.client as mqtt


# ============================================================================
# FOOTPRINT DEFINITIONS
# ============================================================================

FOOTPRINT_ORIGINAL = [
    [0.30, 0.30],
    [-0.30, 0.30],
    [-0.30, -0.30],
    [0.30, -0.30]
]

FOOTPRINT_EXTENDED = [
    [0.50, 0.50],
    [-0.50, 0.50],
    [-0.50, -0.50],
    [0.50, -0.50]
]

FOOTPRINT_NAMES = {
    "original": FOOTPRINT_ORIGINAL,
    "extended": FOOTPRINT_EXTENDED,
}


class TaskState(Enum):
    """Task execution states"""
    IDLE = "IDLE"
    ASSIGNED = "ASSIGNED"
    MOVING_TO_PICKUP = "MOVING_TO_PICKUP"
    ARRIVED_AT_PICKUP = "ARRIVED_AT_PICKUP"
    ATTACHED = "ATTACHED"
    MOVING_TO_DROP = "MOVING_TO_DROP"
    ARRIVED_AT_DROP = "ARRIVED_AT_DROP"
    RELEASED = "RELEASED"
    MOVING_TO_REFERENCE = "MOVING_TO_REFERENCE"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"


class RobotTaskRunner(Node):
    """Main task runner node for autonomous warehouse robot"""

    def __init__(self):
        super().__init__("task_runner")

        # --- ROS2 parameters ---
        self.declare_parameter("robot_id", "robot_1")
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1884)
        self.declare_parameter("mqtt_qos", 1)
        self.declare_parameter("mqtt_reconnect_base", 1.0)   # seconds
        self.declare_parameter("mqtt_reconnect_max", 30.0)   # seconds

        self.robot_id = self.get_parameter("robot_id").value
        self.mqtt_host = self.get_parameter("mqtt_host").value
        self.mqtt_port = int(self.get_parameter("mqtt_port").value)
        self.mqtt_qos = int(self.get_parameter("mqtt_qos").value)
        self._reconnect_base = float(self.get_parameter("mqtt_reconnect_base").value)
        self._reconnect_max = float(self.get_parameter("mqtt_reconnect_max").value)

        self.get_logger().info(f"Task Runner initialized for {self.robot_id}")

        # --- State ---
        self.current_task: Optional[Dict[str, Any]] = None
        self.task_state = TaskState.IDLE
        self.reference_point: Optional[Dict[str, float]] = None

        # --- Nav2 action client ---
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for Nav2 server...")
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("Nav2 server not available yet. Still continuing; will wait on send.")
        else:
            self.get_logger().info("Nav2 server ready")

        # --- ROS2 publisher ---
        self.task_status_pub = self.create_publisher(String, f"/{self.robot_id}/task/status", 10)

        # --- Robot position tracking (new) ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_position_update = time.time()
        self._position_lock = threading.Lock()

        # --- MQTT client setup (paho-mqtt v2.x compatible) ---
        try:
            self.mqtt_client = mqtt.Client(
                client_id=self.robot_id,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1
            )
        except Exception as e:
            # Fallback for older/newer versions (rare)
            self.get_logger().warn(f"mqtt.Client init fallback: {e}")
            self.mqtt_client = mqtt.Client(client_id=self.robot_id)

        # Attach callbacks (use flexible signatures to be compatible)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect

        self._mqtt_connected = False
        self._mqtt_reconnect_attempts = 0
        self._mqtt_lock = threading.Lock()

        # Start connection
        self._start_mqtt()

        # --- Footprint switching setup ---
        self.declare_parameter("robot_namespace", "")
        self.declare_parameter("initial_footprint", "original")
        
        self.robot_namespace = self.get_parameter("robot_namespace").value
        self.initial_footprint = self.get_parameter("initial_footprint").value
        
        # Normalize namespace
        if self.robot_namespace and not self.robot_namespace.startswith('/'):
            self.robot_namespace = '/' + self.robot_namespace
        if self.robot_namespace.endswith('/'):
            self.robot_namespace = self.robot_namespace[:-1]
        
        # Footprint state
        self.current_footprint = self.initial_footprint
        self._footprint_switching = False
        self._footprint_lock = threading.Lock()
        
        # Create footprint publishers
        local_topic = f'{self.robot_namespace}/local_costmap/footprint' if self.robot_namespace else '/local_costmap/footprint'
        global_topic = f'{self.robot_namespace}/global_costmap/footprint' if self.robot_namespace else '/global_costmap/footprint'
        
        self.local_costmap_footprint_pub = self.create_publisher(Polygon, local_topic, 1)
        self.global_costmap_footprint_pub = self.create_publisher(Polygon, global_topic, 1)
        
        # Apply initial footprint
        self._apply_footprint(self.initial_footprint)
        
        self.get_logger().info(f"Footprint switching initialized (initial: {self.initial_footprint})")

        self.get_logger().info("Task Runner ready and waiting for tasks")

    # ---------------- MQTT lifecycle ----------------

    def _start_mqtt(self):
        """Connect to MQTT broker and start network loop"""
        try:
            self.get_logger().info(f"Connecting to MQTT broker {self.mqtt_host}:{self.mqtt_port} ...")
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self.mqtt_client.loop_start()
            # connection result will be handled by on_connect
        except Exception as e:
            self.get_logger().error(f"Initial MQTT connect error: {e}")
            self._schedule_reconnect()

    def _schedule_reconnect(self):
        """Schedule reconnect with exponential backoff"""
        with self._mqtt_lock:
            self._mqtt_reconnect_attempts += 1
            delay = min(self._reconnect_base * (2 ** (self._mqtt_reconnect_attempts - 1)), self._reconnect_max)
            self.get_logger().info(f"Scheduling MQTT reconnect in {delay:.1f}s (attempt {self._mqtt_reconnect_attempts})")
            timer = threading.Timer(delay, self._attempt_reconnect)
            timer.daemon = True
            timer.start()

    def _attempt_reconnect(self):
        """Try to reconnect to MQTT broker"""
        with self._mqtt_lock:
            try:
                self.get_logger().info("Attempting MQTT reconnect...")
                self.mqtt_client.reconnect()
            except Exception as e:
                self.get_logger().warn(f"Reconnect failed: {e}")
                self._schedule_reconnect()

    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT connect callback (compatible signature)"""
        if rc == 0:
            self.get_logger().info("MQTT connected successfully")
            self._mqtt_connected = True
            self._mqtt_reconnect_attempts = 0

            # Subscribe to the topics we need
            try:
                client.subscribe(f"robot/{self.robot_id}/task/assignment", qos=self.mqtt_qos)
                client.subscribe(f"robot/{self.robot_id}/reference_point/update", qos=self.mqtt_qos)
                # optional: admin or broadcast topic
                client.subscribe("robot/all/task/assignment", qos=self.mqtt_qos)
                self.get_logger().info("MQTT subscriptions set")
            except Exception as e:
                self.get_logger().error(f"Subscription failed: {e}")
        else:
            self.get_logger().error(f"MQTT failed to connect, rc={rc}; scheduling reconnect")
            self._schedule_reconnect()

    def _on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        """MQTT disconnect callback (compatible signature)"""
        self._mqtt_connected = False
        # rc == 0: clean disconnect, other values are unexpected
        if rc != 0:
            self.get_logger().warn(f"Unexpected MQTT disconnect (rc={rc}); scheduling reconnect")
            self._schedule_reconnect()
        else:
            self.get_logger().info("MQTT cleanly disconnected")

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            payload = msg.payload.decode("utf-8")
            # Accept either raw JSON or gzipped etc. Here assume JSON string.
            data = json.loads(payload)
        except Exception as e:
            self.get_logger().error(f"Failed to decode MQTT payload on {msg.topic}: {e}")
            return

        topic = msg.topic
        # route topics
        if topic.endswith("/task/assignment") or "task/assignment" in topic:
            self.get_logger().info(f"MQTT Task assignment received on {topic}")
            # allow both direct and wrapped formats
            if isinstance(data, dict):
                self.on_task_assignment(data)
            else:
                self.get_logger().warn("Task assignment payload not an object")
        elif topic.endswith("/reference_point/update") or "reference_point/update" in topic:
            self.get_logger().info("MQTT Reference point update received")
            if isinstance(data, dict):
                self.on_reference_point_update(data)
            else:
                self.get_logger().warn("Reference point payload not an object")
        else:
            self.get_logger().debug(f"MQTT message on unhandled topic {topic}")

    # ---------------- Task handling ----------------

    def on_task_assignment(self, task_data: Dict[str, Any]):
        """Handle a new task assignment"""
        if self.task_state != TaskState.IDLE:
            self.get_logger().warn(f"Robot busy ({self.task_state.value}), ignoring new task")
            # Optionally publish rejection
            self._publish_mqtt({
                "task_id": task_data.get("task_id"),
                "robot_id": self.robot_id,
                "status": "REJECTED_BUSY",
                "reason": f"Robot already in state {self.task_state.value}",
                "timestamp": time.time()
            })
            return

        # Validate minimal fields
        required = ["task_id", "pickup_x", "pickup_y", "drop_x", "drop_y"]
        for f in required:
            if f not in task_data:
                self.get_logger().error(f"Task missing required field: {f}")
                # Publish error
                self._publish_mqtt({
                    "task_id": task_data.get("task_id"),
                    "robot_id": self.robot_id,
                    "status": "ERROR",
                    "reason": f"Missing required field: {f}",
                    "timestamp": time.time()
                })
                return

        self.current_task = task_data
        self.task_state = TaskState.ASSIGNED
        # If the assignment includes a drop zone, set it as the reference point
        # locally so the robot can return to it even if a separate reference
        # MQTT message is missed.
        try:
            if isinstance(task_data, dict) and task_data.get("drop_x") is not None and task_data.get("drop_y") is not None:
                self.on_reference_point_update({
                    "x": float(task_data.get("drop_x", 0.0)),
                    "y": float(task_data.get("drop_y", 0.0)),
                    "yaw": float(task_data.get("drop_yaw", 0.0))
                })
                self.get_logger().info("Reference point set from assignment payload")
        except Exception as e:
            self.get_logger().warn(f"Failed to set reference point from assignment: {e}")

        self.get_logger().info(
            f"Task assigned: {task_data.get('task_id')} | "
            f"Type: {task_data.get('task_type', 'UNKNOWN')} | "
            f"Pickup: ({task_data.get('pickup_x')},{task_data.get('pickup_y')}) | "
            f"Drop: ({task_data.get('drop_x')},{task_data.get('drop_y')})"
        )

        self.publish_status("ASSIGNED")
        # Start navigation to pickup
        self.move_to_pickup()

    def on_reference_point_update(self, point_data: Dict[str, float]):
        """Update reference point"""
        try:
            self.reference_point = {
                "x": float(point_data.get("x", 0.0)),
                "y": float(point_data.get("y", 0.0)),
                "yaw": float(point_data.get("yaw", 0.0)),
            }
            self.get_logger().info(f"Reference point updated: {self.reference_point}")
        except Exception as e:
            self.get_logger().error(f"Invalid reference point payload: {e}")

    # ---------------- Navigation functions ----------------

    def move_to_pickup(self):
        if not self.current_task:
            self.get_logger().warn("move_to_pickup called but no current_task")
            return

        self.task_state = TaskState.MOVING_TO_PICKUP
        self.publish_status("MOVING_TO_PICKUP")

        x = float(self.current_task.get("pickup_x", 0.0))
        y = float(self.current_task.get("pickup_y", 0.0))
        yaw = float(self.current_task.get("pickup_yaw", 0.0))

        self.get_logger().info(f"Moving to pickup: ({x}, {y})")
        goal = self._create_nav_goal(x, y, yaw)

        # ensure nav server available
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 server not ready when sending pickup goal; will try anyway")
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._handle_nav_result(f, TaskState.ARRIVED_AT_PICKUP))

    def move_to_drop(self):
        if not self.current_task:
            self.get_logger().warn("move_to_drop called but no current_task")
            return

        self.task_state = TaskState.MOVING_TO_DROP
        self.publish_status("MOVING_TO_DROP")

        self.switch_to_extended_footprint()

        x = float(self.current_task.get("drop_x", 0.0))
        y = float(self.current_task.get("drop_y", 0.0))
        yaw = float(self.current_task.get("drop_yaw", 0.0))

        self.get_logger().info(f"Moving to drop: ({x}, {y})")
        goal = self._create_nav_goal(x, y, yaw)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 server not ready when sending drop goal; will try anyway")
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._handle_nav_result(f, TaskState.ARRIVED_AT_DROP))

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

        self.get_logger().info("Returning to reference point")
        goal = self._create_nav_goal(x, y, yaw)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 server not ready when sending reference goal; will try anyway")
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._handle_nav_result(f, TaskState.COMPLETED))

    # ---------------- Navigation callbacks ----------------

    def _handle_nav_result(self, future, next_state: TaskState):
        """Handle navigation result (goal accepted -> wait for result)"""
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to get goal handle: {e}")
            self.task_state = TaskState.ERROR
            self.publish_status("ERROR")
            # Publish detailed error
            self._publish_mqtt({
                "task_id": self.current_task.get("task_id") if self.current_task else None,
                "robot_id": self.robot_id,
                "status": "ERROR",
                "reason": f"Failed to get goal handle: {str(e)}",
                "timestamp": time.time()
            }, topic_suffix="task/error")
            return

        if not handle.accepted:
            self.get_logger().error("Nav2 rejected goal")
            self.task_state = TaskState.ERROR
            self.publish_status("ERROR")
            self._publish_mqtt({
                "task_id": self.current_task.get("task_id") if self.current_task else None,
                "robot_id": self.robot_id,
                "status": "ERROR",
                "reason": "Nav2 server rejected goal",
                "timestamp": time.time()
            }, topic_suffix="task/error")
            return

        self.get_logger().info("Nav2 accepted goal, waiting for result...")
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(lambda f: self._handle_nav_complete(f, next_state))

    def _handle_nav_complete(self, future, next_state: TaskState):
        """Handle the navigation completion callback"""
        try:
            # get result (safely)
            res = future.result()
            # res.status should be checked; but here we proceed on success
            self.get_logger().info(f"Reached destination (state={next_state.value})")

            # Update position to reached destination
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

            if next_state == TaskState.ARRIVED_AT_PICKUP:
                self.task_state = TaskState.ATTACHED
                self.publish_status("ATTACHED")
                self.get_logger().info("Shelf attached (simulated). Proceeding to drop.")
                # small delay to simulate attach
                time.sleep(1.0)
                self.move_to_drop()

            elif next_state == TaskState.ARRIVED_AT_DROP:
                self.task_state = TaskState.RELEASED
                self.publish_status("RELEASED")
                self.get_logger().info("Shelf released (simulated). Returning to reference.")
                time.sleep(1.0)
                self.move_to_reference()

            elif next_state == TaskState.COMPLETED:
                self.task_state = TaskState.COMPLETED
                self.publish_status("COMPLETED")
                self.get_logger().info("Task completed ✓")
                self.switch_to_original_footprint()
                self.current_task = None
                self.task_state = TaskState.IDLE

        except Exception as e:
            self.get_logger().error(f"Error handling nav completion: {e}")
            self.task_state = TaskState.ERROR
            self.publish_status("ERROR")
            # Publish detailed error info
            self._publish_mqtt({
                "task_id": self.current_task.get("task_id") if self.current_task else None,
                "robot_id": self.robot_id,
                "status": "ERROR",
                "reason": f"Navigation completion error: {str(e)}",
                "timestamp": time.time()
            }, topic_suffix="task/error")

    # ---------------- Helpers ----------------

    def _create_nav_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        """Create a NavigateToPose goal from x,y,yaw in map frame"""
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        # Convert yaw to quaternion (z,w)
        qz = math.sin(float(yaw) / 2.0)
        qw = math.cos(float(yaw) / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose
        return goal

    def publish_status(self, status: str):
        """Publish task status to ROS2 topic and MQTT (with extended tracking)"""
        msg_data = {
            "task_id": self.current_task.get("task_id") if self.current_task else None,
            "robot_id": self.robot_id,
            "status": status,
            "task_type": self.current_task.get("task_type") if self.current_task else None,
            "task_state": self.task_state.value,
            "timestamp": time.time(),
        }

        # Include current progress if available
        if self.current_task:
            msg_data.update({
                "shelf_id": self.current_task.get("shelf_id"),
                "priority": self.current_task.get("priority"),
                "pickup_location": {
                    "x": float(self.current_task.get("pickup_x", 0.0)),
                    "y": float(self.current_task.get("pickup_y", 0.0))
                },
                "drop_location": {
                    "x": float(self.current_task.get("drop_x", 0.0)),
                    "y": float(self.current_task.get("drop_y", 0.0))
                }
            })

        # ROS2
        ros_msg = String()
        ros_msg.data = json.dumps(msg_data)
        try:
            self.task_status_pub.publish(ros_msg)
        except Exception as e:
            self.get_logger().debug(f"Failed to publish ROS2 status: {e}")

        # MQTT (publish to both task status and live progress)
        self._publish_mqtt(msg_data)
        
        # Also publish live progress update for real-time frontend tracking
        try:
            self._publish_mqtt({
                "task_id": msg_data.get("task_id"),
                "robot_id": self.robot_id,
                "task_state": self.task_state.value,
                "status": status,
                "timestamp": time.time()
            }, topic_suffix="task/progress")
        except Exception as e:
            self.get_logger().debug(f"Failed to publish task progress: {e}")

    def _publish_mqtt(self, payload: Dict[str, Any], topic_suffix: str = "task/status"):
        """Publish JSON payload to robot/<id>/<topic_suffix> with QoS and simple error handling"""
        try:
            topic = f"robot/{self.robot_id}/{topic_suffix}"
            payload_str = json.dumps(payload)
            self.mqtt_client.publish(topic, payload_str, qos=self.mqtt_qos)
            self.get_logger().debug(f"Published MQTT {topic}: {payload_str}")
        except Exception as e:
            self.get_logger().error(f"MQTT publish failed: {e}")

    def update_robot_position(self, x: float, y: float, yaw: float = 0.0):
        """Update robot's current position and publish to backend (for real-time tracking)"""
        with self._position_lock:
            self.robot_x = float(x)
            self.robot_y = float(y)
            self.robot_yaw = float(yaw)
            self.last_position_update = time.time()
        
        # Publish position update to MQTT for backend tracking
        try:
            self._publish_mqtt({
                "robot_id": self.robot_id,
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
                "timestamp": time.time()
            }, topic_suffix="position/update")
        except Exception as e:
            self.get_logger().debug(f"Failed to publish position update: {e}")
        
        # If currently carrying a shelf during task, publish shelf location too
        if self.current_task and self.task_state in [
            TaskState.ATTACHED,
            TaskState.MOVING_TO_DROP,
            TaskState.ARRIVED_AT_DROP
        ]:
            try:
                shelf_id = self.current_task.get("shelf_id")
                if shelf_id:
                    self._publish_mqtt({
                        "shelf_id": shelf_id,
                        "robot_id": self.robot_id,
                        "x": float(x),
                        "y": float(y),
                        "yaw": float(yaw),
                        "timestamp": time.time()
                    }, topic_suffix="shelf/location")
            except Exception as e:
                self.get_logger().debug(f"Failed to publish shelf location: {e}")

    # ---------------- Footprint Switching ----------------

    def _create_polygon_msg(self, points: list) -> Polygon:
        """Create a Polygon message from list of points"""
        polygon = Polygon()
        for point in points:
            p = Point32()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.0
            polygon.points.append(p)
        return polygon

    def _apply_footprint(self, footprint_name: str) -> bool:
        """Apply footprint configuration"""
        if footprint_name not in FOOTPRINT_NAMES:
            self.get_logger().error(f"❌ Unknown footprint: {footprint_name}")
            return False
        
        with self._footprint_lock:
            if self._footprint_switching:
                self.get_logger().warn("⚠️  Footprint switch already in progress")
                return False
            self._footprint_switching = True
        
        try:
            footprint_points = FOOTPRINT_NAMES[footprint_name]
            polygon_msg = self._create_polygon_msg(footprint_points)
            
            # Publish to costmaps
            self.local_costmap_footprint_pub.publish(polygon_msg)
            self.global_costmap_footprint_pub.publish(polygon_msg)
            
            # Update internal state
            self.current_footprint = footprint_name
            
            self.get_logger().info(f"✅ Footprint '{footprint_name}' applied")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error applying footprint: {str(e)}")
            return False
        finally:
            with self._footprint_lock:
                self._footprint_switching = False

    def switch_to_extended_footprint(self) -> bool:
        """Switch to extended footprint (called when moving to drop-off)"""
        if self.current_footprint == "extended":
            self.get_logger().debug("ℹ️  Already using extended footprint")
            return True
        
        self.get_logger().info("👣 Switching to EXTENDED footprint for drop-off navigation")
        return self._apply_footprint("extended")

    def switch_to_original_footprint(self) -> bool:
        """Switch to original footprint (called after task completion)"""
        if self.current_footprint == "original":
            self.get_logger().debug("ℹ️  Already using original footprint")
            return True
        
        self.get_logger().info("👣 Switching back to ORIGINAL footprint")
        return self._apply_footprint("original")

    # ---------------- Shutdown ----------------

    def destroy_node(self):
        """Graceful shutdown override to stop MQTT loop too"""
        try:
            if hasattr(self, "mqtt_client"):
                try:
                    # try clean disconnect
                    self.mqtt_client.disconnect()
                except Exception:
                    pass
                try:
                    self.mqtt_client.loop_stop()
                except Exception:
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
        # graceful teardown
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
