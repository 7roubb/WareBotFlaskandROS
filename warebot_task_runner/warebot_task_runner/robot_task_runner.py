"""
WareBot Task Runner - Main Integration Class with Built-in Footprint Switching
MODIFIED FOR: Calibration file path parameter from launch (ONLY calibration.npy)
"""
import json
import time
import threading
from typing import Optional, Dict, Any, List
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32
from std_srvs.srv import Trigger

from warebot_task_runner.enums import TaskState
from warebot_task_runner.constants import *
from warebot_task_runner.controllers import LinearActuatorController
from warebot_task_runner.core import TaskManager, MQTTHandler, NavigationHandler, AlignmentController


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


class DummyAlignmentController:
    """Mock controller for simulation"""
    def __init__(self, logger):
        self.logger = logger
        self.callback = None
    def set_aligned_callback(self, callback):
        self.callback = callback
    def set_actuator_controller(self, controller):
        pass
    def start_alignment(self):
        self.logger.info("🤖 [SIM] Alignment started (Mock)")
        # Simulate delay then verify
        if self.callback:
            # Call immediately or use a timer if needed, but for simplicity:
            threading.Timer(2.0, self._finish).start()
    def _finish(self):
        self.logger.info("🤖 [SIM] Alignment complete (Mock)")
        if self.callback: self.callback()

class DummyActuatorController:
    """Mock controller for simulation"""
    def __init__(self, logger):
        self.logger = logger
    def extend(self):
        self.logger.info("🤖 [SIM] Actuators EXTENDING (Mock)")
        return True
    def retract(self):
        self.logger.info("🤖 [SIM] Actuators RETRACTING (Mock)")
        return True
    def stop(self):
        pass


class RobotTaskRunner(Node):

    """
    Integrated Task Runner with AprilTag Alignment + MQTT + Linear Actuators + Built-in Footprint Switching
    
    MODIFIED: Simple calibration_file_path parameter from launch
    - No separate matrix/coeffs files
    - Uses only calibration.npy
    - Path provided via launch parameter
    """

    def __init__(self):
        super().__init__("task_runner")

        # Declare ROS2 parameters
        self._declare_parameters()
        
        # Get parameter values
        self._load_parameters()

        # Fleet State for assignment logic
        self.fleet_state = {}
        self.fleet_lock = threading.Lock()

        self.get_logger().info(f"Task Runner initialized for {self.robot_id}")

        # Robot position tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_position_update = time.time()
        self._position_lock = threading.Lock()
        
        # Reference point
        self.reference_point: Optional[Dict[str, float]] = None

        # Initialize Task Manager
        self.task_manager = TaskManager(self.get_logger())

        # Initialize Navigation Handler
        self.nav_handler = NavigationHandler(self, self.get_logger())

        # ========================================================================
        # MODIFIED: Initialize Alignment Controller with calibration_file_path
        # ========================================================================
        
        if self.enable_hardware:
            self.alignment_controller = AlignmentController(
                node=self,
                logger=self.get_logger(),
                cmd_vel_topic=self.cmd_vel_topic,
                image_topic=self.image_topic,
                estop_topic=self.estop_topic,
                show_debug=self.show_debug,
                lost_tag_timeout=self.lost_tag_timeout,
                alignment_hold_time=self.alignment_hold_time,
                calibration_file_path=self.calibration_file_path,  # From launch parameter
                tag_size=0.068,
                tag_ids=[294, 323, 420, 530],
                package_share_dir=None  # Will auto-detect
            )
            # Initialize Linear Actuator Controller
            self.actuator_controller = LinearActuatorController(
                port=self.actuator_port,
                baudrate=self.actuator_baudrate,
                logger=self.get_logger()
            )
        else:
            self.get_logger().warn("⚠️  SIMULATION MODE: Hardware Disabled")
            self.alignment_controller = DummyAlignmentController(self.get_logger())
            self.actuator_controller = DummyActuatorController(self.get_logger())

        self.alignment_controller.set_aligned_callback(self.on_alignment_complete)
        self.alignment_controller.set_actuator_controller(self.actuator_controller)

        # ✅ Initialize Built-in Footprint Switcher
        self._init_footprint_switcher()

        # Initialize MQTT Handler
        self.mqtt_handler = MQTTHandler(
            robot_id=self.robot_id,
            host=self.mqtt_host,
            port=self.mqtt_port,
            qos=self.mqtt_qos,
            reconnect_base=self.reconnect_base,
            reconnect_max=self.reconnect_max,
            logger=self.get_logger(),
            on_task_assignment=self.on_task_assignment,
            on_reference_update=self.on_reference_point_update,
            on_fleet_status=self.on_fleet_status
        )
        self.mqtt_handler.start()

        # ROS2 publishers
        self.task_status_pub = self.create_publisher(
            String, f"/{self.robot_id}/task/status", 10
        )

        # Status publisher timer
        self.status_timer = self.create_timer(
            STATUS_PUBLISH_INTERVAL, 
            self.process_status_queue
        )

        self._log_startup_info()

    def _declare_parameters(self):
        """Declare all ROS2 parameters"""
        self.declare_parameter("robot_id", "robot_1")
        self.declare_parameter("enable_hardware", True)  # NEW: For simulation mode
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1884)
        self.declare_parameter("mqtt_qos", 1)
        self.declare_parameter("mqtt_reconnect_base", 1.0)
        self.declare_parameter("mqtt_reconnect_max", 30.0)
        
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("estop_topic", "/emergency_stop_state")
        self.declare_parameter("show_debug_image", True)
        self.declare_parameter("lost_tag_timeout", 0.7)
        
        # ========================================================================
        # MODIFIED: ONLY calibration_file_path parameter (no separate matrix/coeffs)
        # ========================================================================
        self.declare_parameter("calibration_file_path", "")  # Path to calibration.npy
        self.declare_parameter("alignment_hold_time", 30.0)
        
        self.declare_parameter("actuator_port", "/dev/ttyUSB1")
        self.declare_parameter("actuator_baudrate", 115200)
        
        # ✅ Footprint parameters
        self.declare_parameter("footprint_service", "/switch_footprint")
        self.declare_parameter("initial_footprint", "original")
        self.declare_parameter("robot_namespace", "")

    def _load_parameters(self):
        """Load parameter values"""
        self.robot_id = self.get_parameter("robot_id").value
        self.enable_hardware = self.get_parameter("enable_hardware").value
        self.mqtt_host = self.get_parameter("mqtt_host").value
        self.mqtt_port = int(self.get_parameter("mqtt_port").value)
        self.mqtt_qos = int(self.get_parameter("mqtt_qos").value)
        self.reconnect_base = float(self.get_parameter("mqtt_reconnect_base").value)
        self.reconnect_max = float(self.get_parameter("mqtt_reconnect_max").value)
        
        self.image_topic = self.get_parameter("image_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.estop_topic = self.get_parameter("estop_topic").value
        self.show_debug = self.get_parameter("show_debug_image").value
        self.lost_tag_timeout = self.get_parameter("lost_tag_timeout").value
        self.alignment_hold_time = self.get_parameter("alignment_hold_time").value
        
        # ========================================================================
        # MODIFIED: Load calibration_file_path from launch parameter
        # ========================================================================
        self.calibration_file_path = self.get_parameter("calibration_file_path").value
        
        # Log calibration configuration
        self.get_logger().info("=" * 70)
        self.get_logger().info("⚙️  CALIBRATION CONFIGURATION")
        self.get_logger().info("=" * 70)
        if self.calibration_file_path and self.calibration_file_path.strip():
            self.get_logger().info(f"📄 Using calibration file:")
            self.get_logger().info(f"   Path: {self.calibration_file_path}")
        else:
            self.get_logger().info(f"📄 No calibration file path specified in launch parameter")
            self.get_logger().info(f"   Will search standard locations:")
            self.get_logger().info(f"   - config/calibration.npy")
            self.get_logger().info(f"   - data/calibration/calibration.npy")
        
        self.actuator_port = self.get_parameter("actuator_port").value
        self.actuator_baudrate = int(self.get_parameter("actuator_baudrate").value)
        
        # ✅ Footprint parameters
        self.footprint_service_name = self.get_parameter("footprint_service").value
        self.initial_footprint = self.get_parameter("initial_footprint").value
        self.robot_namespace = self.get_parameter("robot_namespace").value

    # ========================================================================
    # BUILT-IN FOOTPRINT SWITCHER
    # ========================================================================

    def _init_footprint_switcher(self):
        """Initialize built-in footprint switching functionality"""
        self.current_footprint = self.initial_footprint
        self._footprint_switching = False
        self._footprint_lock = threading.Lock()
        
        # Normalize namespace
        if self.robot_namespace and not self.robot_namespace.startswith('/'):
            self.robot_namespace = '/' + self.robot_namespace
        if self.robot_namespace.endswith('/'):
            self.robot_namespace = self.robot_namespace[:-1]
        
        # Create footprint service
        self.footprint_service = self.create_service(
            Trigger,
            self.footprint_service_name,
            self._handle_footprint_switch,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        # Publishers for footprint updates
        local_topic = f'{self.robot_namespace}/local_costmap/footprint' if self.robot_namespace else '/local_costmap/footprint'
        global_topic = f'{self.robot_namespace}/global_costmap/footprint' if self.robot_namespace else '/global_costmap/footprint'
        status_topic = f'{self.robot_namespace}/footprint_status' if self.robot_namespace else '/footprint_status'
        
        self.local_costmap_footprint_pub = self.create_publisher(Polygon, local_topic, 1)
        self.global_costmap_footprint_pub = self.create_publisher(Polygon, global_topic, 1)
        self.footprint_status_pub = self.create_publisher(String, status_topic, 1)
        
        # Apply initial footprint
        self._apply_footprint(self.initial_footprint)
        
        self.get_logger().info(f"✅ Built-in footprint switcher initialized")
        self.get_logger().info(f"   Service: {self.footprint_service_name}")
        self.get_logger().info(f"   Initial footprint: {self.initial_footprint}")

    def _handle_footprint_switch(self, request, response):
        """Handle footprint switch service call"""
        new_footprint = "extended" if self.current_footprint == "original" else "original"
        
        success = self._apply_footprint(new_footprint)
        
        response.success = success
        if success:
            response.message = f"✅ Footprint switched to {new_footprint}"
            self.get_logger().info(f"🔄 Footprint switched to {new_footprint}")
        else:
            response.message = f"❌ Failed to switch footprint"
            self.get_logger().error(f"❌ Failed to switch footprint")
        
        return response

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
            
            # Publish status
            status_msg = String()
            status_msg.data = json.dumps({
                "footprint": footprint_name,
                "points": footprint_points,
                "width": max([abs(p[0]) for p in footprint_points]) * 2,
                "height": max([abs(p[1]) for p in footprint_points]) * 2,
                "timestamp": self.get_clock().now().nanoseconds
            })
            self.footprint_status_pub.publish(status_msg)
            
            self.get_logger().info(f"✅ Footprint '{footprint_name}' applied")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error applying footprint: {str(e)}")
            return False
        finally:
            with self._footprint_lock:
                self._footprint_switching = False

    def _create_polygon_msg(self, points: List[List[float]]) -> Polygon:
        """Create a Polygon message from list of points"""
        polygon = Polygon()
        for point in points:
            p = Point32()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.0
            polygon.points.append(p)
        return polygon

    def switch_to_extended_footprint(self) -> bool:
        """Switch to extended footprint (called when actuators extend)"""
        if self.current_footprint == "extended":
            self.get_logger().debug("ℹ️  Already using extended footprint")
            return True
        
        self.get_logger().info("👣 Switching to EXTENDED footprint")
        return self._apply_footprint("extended")

    def switch_to_original_footprint(self) -> bool:
        """Switch to original footprint (called when actuators retract)"""
        if self.current_footprint == "original":
            self.get_logger().debug("ℹ️  Already using original footprint")
            return True
        
        self.get_logger().info("👣 Switching to ORIGINAL footprint")
        return self._apply_footprint("original")

    # ========================================================================
    # STARTUP AND LOGGING
    # ========================================================================

    def _log_startup_info(self):
        """Log startup information"""
        self.get_logger().info("=" * 70)
        self.get_logger().info("🤖 Task Runner with MQTT + Actuators + Built-in Footprint Started")
        self.get_logger().info(f"📷 Image topic: {self.image_topic}")
        self.get_logger().info(f"🎮 Cmd vel topic: {self.cmd_vel_topic}")
        self.get_logger().info(f"📡 MQTT: {self.mqtt_host}:{self.mqtt_port}")
        self.get_logger().info(f"🔧 Actuators: {self.actuator_port} @ {self.actuator_baudrate} baud")
        self.get_logger().info(f"👣 Footprint service: {self.footprint_service_name}")
        self.get_logger().info("=" * 70)

    # ================= TASK HANDLING =================

    def on_fleet_status(self, robot_name: str, data: Dict[str, Any]):
        """Update fleet state"""
        with self.fleet_lock:
            self.fleet_state[robot_name] = data

    def on_task_assignment(self, task_data: Dict[str, Any], is_broadcast: bool = False):
        """Handle task assignment with deduplication"""
        task_id = task_data.get("task_id")
        
        if not task_id:
            self.get_logger().error("❌ Task has no task_id, rejecting")
            return
        
        # Check if duplicate
        if self.task_manager.is_task_duplicate(task_id):
            self.get_logger().warn(f"⚠️  DUPLICATE task assignment received and IGNORED: {task_id}")
            return
        
        if self.task_manager.task_state != TaskState.IDLE:
            self.get_logger().warn(
                f"⚠️  Robot busy ({self.task_manager.task_state.value}), queuing task: {task_id}"
            )
            self.task_manager.queue_task(task_data)
            
            self.mqtt_handler.publish_direct({
                "task_id": task_id,
                "robot_id": self.robot_id,
                "status": "QUEUED",
                "reason": f"Robot in state {self.task_manager.task_state.value}",
                "timestamp": time.time()
            }, "task/status")
            return
            
        # Assignment Logic (Nearest Neighbor if broadcast)
        if is_broadcast:
            should_accept = self._check_assignment_criteria(task_data)
            if not should_accept:
                return

        # Execute task immediately
        self._execute_task(task_data)
        
    def _check_assignment_criteria(self, task_data: Dict[str, Any]) -> bool:
        """Determines if this robot should accept the task based on nearest neighbor logic"""
        
        # 1. Calculate my distance
        if not self.reference_point: 
            # If no ref point, assume we are at (0,0) or current position?
            # Use current robot position
            my_x, my_y = self.robot_x, self.robot_y
        else:
            my_x, my_y = self.robot_x, self.robot_y
            
        pickup_x = float(task_data.get("pickup_x", 0.0))
        pickup_y = float(task_data.get("pickup_y", 0.0))
        
        my_dist = math.hypot(my_x - pickup_x, my_y - pickup_y)
        
        # 2. Check other IDLE robots
        # We need to know who is IDLE and where they are
        
        best_robot = self.robot_id
        min_dist = my_dist
        
        with self.fleet_lock:
            for r_name, r_data in self.fleet_state.items():
                if r_name == self.robot_id:
                    continue # Skip self (already calc)
                    
                status = r_data.get("status", "IDLE")
                # Only compare with IDLE robots
                if status == "IDLE":
                    r_x = float(r_data.get("x", 0.0))
                    r_y = float(r_data.get("y", 0.0))
                    r_dist = math.hypot(r_x - pickup_x, r_y - pickup_y)
                    
                    if r_dist < min_dist:
                        min_dist = r_dist
                        best_robot = r_name
        
        if best_robot != self.robot_id:
            self.get_logger().info(f"🚫 Ignoring task {task_data.get('task_id')}: Robot {best_robot} is closer ({min_dist:.2f}m vs {my_dist:.2f}m)")
            return False
            
        self.get_logger().info(f"✅ Accepting task {task_data.get('task_id')}: checking in as nearest robot ({my_dist:.2f}m)")
        return True

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

    def _execute_task(self, task_data: Dict[str, Any]):
        """Execute a task"""
        task_id = task_data.get("task_id")
        
        # Mark as in progress
        self.task_manager.mark_task_in_progress(task_id)
        
        # Validate required fields
        required = ["task_id", "pickup_x", "pickup_y", "drop_x", "drop_y"]
        for f in required:
            if f not in task_data:
                self.get_logger().error(f"❌ Task missing field: {f}")
                self.mqtt_handler.publish_direct({
                    "task_id": task_id,
                    "robot_id": self.robot_id,
                    "status": "ERROR",
                    "reason": f"Missing field: {f}",
                    "timestamp": time.time()
                }, "task/status")
                
                self.task_manager.mark_task_completed(task_id)
                self.task_manager.task_state = TaskState.IDLE
                self._process_next_task()
                return

        self.task_manager.current_task = task_data
        self.task_manager.task_state = TaskState.ASSIGNED

        # Set reference point
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
            f"📋 Task assigned: {task_id} | "
            f"Pickup: ({task_data.get('pickup_x')},{task_data.get('pickup_y')}) | "
            f"Drop: ({task_data.get('drop_x')},{task_data.get('drop_y')})"
        )

        self._queue_status_update("ASSIGNED")
        self.move_to_pickup()

    def _process_next_task(self):
        """Process next task from queue"""
        if self.task_manager.task_state != TaskState.IDLE:
            self.get_logger().debug(
                f"⚠️  Cannot process next task - robot in state: {self.task_manager.task_state.value}"
            )
            return
        
        next_task = self.task_manager.get_next_task()
        if not next_task:
            self.get_logger().debug("📭 Task queue empty")
            return
        
        # Final duplicate check
        task_id = next_task.get("task_id")
        if self.task_manager.is_task_duplicate(task_id):
            self.get_logger().warn(f"⚠️  DUPLICATE task found in queue, skipping: {task_id}")
            self._process_next_task()
            return
        
        self.get_logger().info(f"▶️  Processing task from queue: {task_id}")
        self._execute_task(next_task)

    # ================= NAVIGATION =================

    def move_to_pickup(self):
        """Navigate to pickup location"""
        if not self.task_manager.current_task:
            self.get_logger().warn("No current task")
            return

        self.task_manager.task_state = TaskState.MOVING_TO_PICKUP
        self._queue_status_update("MOVING_TO_PICKUP")

        x = float(self.task_manager.current_task.get("pickup_x", 0.0))
        y = float(self.task_manager.current_task.get("pickup_y", 0.0))
        yaw = float(self.task_manager.current_task.get("pickup_yaw", 0.0))

        self.get_logger().info(f"🚀 Moving to pickup: ({x}, {y})")
        self.nav_handler.navigate_to(x, y, yaw, self._handle_pickup_arrival)

    def move_to_drop(self):
        """Navigate to drop location"""
        if not self.task_manager.current_task:
            self.get_logger().warn("No current task")
            return

        self.task_manager.task_state = TaskState.MOVING_TO_DROP
        self._queue_status_update("MOVING_TO_DROP")
        
        # ✅ Switch to extended footprint using built-in method
        self.get_logger().info("=" * 60)
        self.get_logger().info("👣 Switching to EXTENDED footprint for drop-off navigation")
        self.get_logger().info("=" * 60)
        footprint_success = self.switch_to_extended_footprint()
        if footprint_success:
            self.get_logger().info("✅ Footprint switched to EXTENDED successfully")
        else:
            self.get_logger().warn("⚠️  Footprint switch failed")

        x = float(self.task_manager.current_task.get("drop_x", 0.0))
        y = float(self.task_manager.current_task.get("drop_y", 0.0))
        yaw = float(self.task_manager.current_task.get("drop_yaw", 0.0))

        self.get_logger().info(f"🚀 Moving to drop: ({x}, {y})")
        self.nav_handler.navigate_to(x, y, yaw, self._handle_drop_arrival)

    def move_to_reference(self):
        """Return to reference point"""
        if not self.reference_point:
            self.get_logger().warn("Reference point not set")
            self.task_manager.task_state = TaskState.ERROR
            self._queue_status_update("ERROR")
            return

        self.task_manager.task_state = TaskState.MOVING_TO_REFERENCE
        self._queue_status_update("MOVING_TO_REFERENCE")

        x = self.reference_point["x"]
        y = self.reference_point["y"]
        yaw = self.reference_point["yaw"]

        self.get_logger().info("🏠 Returning to reference point")
        self.nav_handler.navigate_to(x, y, yaw, self._handle_reference_arrival)

    # ================= NAVIGATION CALLBACKS =================

    def _handle_pickup_arrival(self, future):
        """Handle arrival at pickup location"""
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal failed: {e}")
            self._handle_navigation_error()
            return

        if not handle.accepted:
            self.get_logger().error("Nav2 rejected goal")
            self._handle_navigation_error()
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
                float(self.task_manager.current_task.get("pickup_x", 0.0)),
                float(self.task_manager.current_task.get("pickup_y", 0.0)),
                float(self.task_manager.current_task.get("pickup_yaw", 0.0))
            )
        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            self._handle_navigation_error()
            return

        self.on_arrived_pickup()

    def _handle_drop_arrival(self, future):
        """Handle arrival at drop location"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("📍 _handle_drop_arrival callback triggered")
        self.get_logger().info("=" * 60)
        
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Goal failed: {e}")
            self._handle_navigation_error()
            return

        if not handle.accepted:
            self.get_logger().error("❌ Nav2 rejected goal")
            self._handle_navigation_error()
            return

        self.get_logger().info("✅ Nav2 drop goal accepted, waiting for result...")
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._on_drop_reached)

    def _on_drop_reached(self, future):
        """Reached drop location"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("📍 _on_drop_reached callback triggered")
        self.get_logger().info("=" * 60)
        
        try:
            res = future.result()
            self.get_logger().info("✅ Reached drop location")
            self.update_robot_position(
                float(self.task_manager.current_task.get("drop_x", 0.0)),
                float(self.task_manager.current_task.get("drop_y", 0.0)),
                float(self.task_manager.current_task.get("drop_yaw", 0.0))
            )
        except Exception as e:
            self.get_logger().error(f"❌ Navigation error: {e}")
            self._handle_navigation_error()
            return

        # Update state
        self.task_manager.task_state = TaskState.ARRIVED_AT_DROP
        self._queue_status_update("ARRIVED_AT_DROP")
        
        # Retract actuators
        self.get_logger().info("=" * 60)
        self.get_logger().info("⬇️  RELEASING SHELF AT DROP LOCATION")
        self.get_logger().info("=" * 60)
        
        retract_success = self.actuator_controller.retract()
        if retract_success:
            self.get_logger().info("✅ Actuator RETRACT command sent")
        else:
            self.get_logger().warn("⚠️  Actuator RETRACT failed")
        
        # ✅ Switch back to original footprint using built-in method
        self.get_logger().info("=" * 60)
        self.get_logger().info("👣 Switching back to ORIGINAL footprint after retraction")
        self.get_logger().info("=" * 60)
        footprint_success = self.switch_to_original_footprint()
        if footprint_success:
            self.get_logger().info("✅ Footprint switched to ORIGINAL")
        else:
            self.get_logger().warn("⚠️  Footprint switch failed")
        
        # Wait for retraction
        self.get_logger().info(f"⏳ Waiting {ACTUATOR_RETRACTION_TIME} seconds for actuator retraction...")
        time.sleep(ACTUATOR_RETRACTION_TIME)
        self.get_logger().info("✅ Wait complete")
        
        self.task_manager.task_state = TaskState.RELEASED
        self._queue_status_update("RELEASED")
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ SHELF RELEASED SUCCESSFULLY")
        self.get_logger().info("=" * 60)
        
        time.sleep(1.0)
        self.move_to_reference()

    def _handle_reference_arrival(self, future):
        """Handle arrival at reference point"""
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal failed: {e}")
            self._handle_navigation_error()
            return

        if not handle.accepted:
            self.get_logger().error("Nav2 rejected goal")
            self._handle_navigation_error()
            return

        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._on_reference_reached)

    def _on_reference_reached(self, future):
        """Reached reference point - task complete"""
        try:
            res = future.result()
            self.get_logger().info("🏠 Returned to reference point")
            if self.reference_point:
                self.update_robot_position(
                    self.reference_point["x"],
                    self.reference_point["y"],
                    self.reference_point["yaw"]
                )
        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")
            self._handle_navigation_error()
            return

        self.task_manager.task_state = TaskState.COMPLETED
        self._queue_status_update("COMPLETED")
        self.get_logger().info("✅ Task completed successfully")
        
        # Mark task as completed
        if self.task_manager.current_task:
            self.task_manager.mark_task_completed(
                self.task_manager.current_task.get("task_id")
            )
        
        self.task_manager.current_task = None
        self.task_manager.task_state = TaskState.IDLE
        
        # Process next task
        self._process_next_task()

    def _handle_navigation_error(self):
        """Handle navigation errors"""
        self.task_manager.task_state = TaskState.ERROR
        self._queue_status_update("ERROR")
        
        if self.task_manager.current_task:
            self.task_manager.mark_task_completed(
                self.task_manager.current_task.get("task_id")
            )
        
        self.task_manager.current_task = None
        self.task_manager.task_state = TaskState.IDLE
        self._process_next_task()

    # ================= ALIGNMENT =================

    def on_arrived_pickup(self):
        """Start AprilTag alignment"""
        self.task_manager.task_state = TaskState.ALIGNING
        self._queue_status_update("ALIGNING")
        self.alignment_controller.start_alignment()

    def on_alignment_complete(self):
        """Called when alignment is complete"""
        self.task_manager.task_state = TaskState.ALIGNED
        self._queue_status_update("ALIGNED")
        self.get_logger().info("✅ Alignment complete - proceeding to drop location")
        self.move_to_drop()

    # ================= STATUS PUBLISHING =================

    def _queue_status_update(self, status: str):
        """Queue a status update"""
        task_id = None
        if self.task_manager.current_task:
            task_id = self.task_manager.current_task.get("task_id")
        
        self.mqtt_handler.queue_status_update(status, task_id)

    def process_status_queue(self):
        """Process status update queue with retries"""
        with self.mqtt_handler.status_publish_lock:
            if not self.mqtt_handler.status_update_queue:
                return
            
            status_item = self.mqtt_handler.status_update_queue[0]
        
        try:
            if not self.mqtt_handler.is_connected():
                self.get_logger().debug("⚠️  MQTT not connected, retrying status publish...")
                status_item["retry_count"] += 1
                if status_item["retry_count"] > MAX_STATUS_RETRIES:
                    with self.mqtt_handler.status_publish_lock:
                        self.mqtt_handler.status_update_queue.popleft()
                    self.get_logger().warn(f"⚠️  Gave up publishing status after {MAX_STATUS_RETRIES} retries")
                return
            
            # Build status message
            msg_data = {
                "task_id": status_item.get("task_id"),
                "robot_id": self.robot_id,
                "status": status_item["status"],
                "task_state": self.task_manager.task_state.value,
                "timestamp": status_item["timestamp"],
            }

            if self.task_manager.current_task:
                msg_data.update({
                    "shelf_id": self.task_manager.current_task.get("shelf_id"),
                    "pickup_location": {
                        "x": float(self.task_manager.current_task.get("pickup_x", 0.0)),
                        "y": float(self.task_manager.current_task.get("pickup_y", 0.0))
                    },
                    "drop_location": {
                        "x": float(self.task_manager.current_task.get("drop_x", 0.0)),
                        "y": float(self.task_manager.current_task.get("drop_y", 0.0))
                    }
                })

            # Publish to ROS2
            ros_msg = String()
            ros_msg.data = json.dumps(msg_data)
            try:
                self.task_status_pub.publish(ros_msg)
            except Exception as e:
                self.get_logger().debug(f"ROS2 publish failed: {e}")

            # Publish to MQTT
            self.mqtt_handler.publish_with_retry(msg_data, status_item)
            
            # Remove from queue
            with self.mqtt_handler.status_publish_lock:
                if (self.mqtt_handler.status_update_queue and 
                    self.mqtt_handler.status_update_queue[0] == status_item):
                    self.mqtt_handler.status_update_queue.popleft()
            
            self.get_logger().info(
                f"📤 Status published: {status_item['status']} "
                f"(attempt {status_item['retry_count'] + 1})"
            )

        except Exception as e:
            self.get_logger().error(f"Error processing status queue: {e}")
            status_item["retry_count"] += 1
            if status_item["retry_count"] > MAX_STATUS_RETRIES:
                with self.mqtt_handler.status_publish_lock:
                    if (self.mqtt_handler.status_update_queue and 
                        self.mqtt_handler.status_update_queue[0] == status_item):
                        self.mqtt_handler.status_update_queue.popleft()

    # ================= HELPERS =================

    def update_robot_position(self, x: float, y: float, yaw: float = 0.0):
        """Update robot position"""
        with self._position_lock:
            self.robot_x = float(x)
            self.robot_y = float(y)
            self.robot_yaw = float(yaw)
            self.last_position_update = time.time()

    def destroy_node(self):
        """Clean shutdown"""
        # Stop alignment
        if hasattr(self, 'alignment_controller'):
            self.alignment_controller.stop_alignment()
        
        # Stop status timer
        if hasattr(self, 'status_timer') and self.status_timer:
            self.status_timer.cancel()

        # Close actuators
        if hasattr(self, 'actuator_controller'):
            try:
                self.actuator_controller.close()
            except:
                pass

        # Stop MQTT
        if hasattr(self, 'mqtt_handler'):
            try:
                self.mqtt_handler.stop()
            except:
                pass

        super().destroy_node()