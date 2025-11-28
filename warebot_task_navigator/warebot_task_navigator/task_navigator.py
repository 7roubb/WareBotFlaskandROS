"""
Task Navigator Node - Main coordinator for robot task navigation
Receives task assignments from backend via MQTT/HTTP
Commands robots to navigate to target locations and back to reference point
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Twist
from std_msgs.msg import String, Float32MultiArray
import json
import logging
from typing import Dict, List, Optional
from dataclasses import dataclass
from enum import Enum
import math
import time


class TaskStatus(Enum):
    """Task status enumeration"""
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    RETURNING = "RETURNING"


class RobotState(Enum):
    """Robot state enumeration"""
    IDLE = "IDLE"
    MOVING_TO_TARGET = "MOVING_TO_TARGET"
    AT_TARGET = "AT_TARGET"
    RETURNING_TO_HOME = "RETURNING_TO_HOME"
    AT_HOME = "AT_HOME"
    ERROR = "ERROR"


@dataclass
class TaskAssignment:
    """Task assignment data structure"""
    task_id: str
    robot_id: str
    target_x: float
    target_y: float
    target_yaw: float = 0.0
    priority: int = 0
    timestamp: float = 0.0


@dataclass
class ReferencePoint:
    """Reference point (home/dock location) for robots"""
    robot_id: str
    x: float
    y: float
    yaw: float = 0.0


class TaskNavigator(Node):
    """
    Main coordinator node for robot task navigation
    - Receives task assignments from backend
    - Manages robot movements to task locations
    - Handles return to reference points
    - Tracks task and robot states
    """

    def __init__(self):
        super().__init__('task_navigator')
        
        self.logger = logging.getLogger('TaskNavigator')
        self.get_logger().info('Initializing Task Navigator Node')

        # Configuration
        self.declare_parameter('backend_host', 'localhost')
        self.declare_parameter('backend_port', 5000)
        self.declare_parameter('robot_max_speed', 0.5)  # m/s
        self.declare_parameter('tolerance_distance', 0.1)  # meters
        self.declare_parameter('tolerance_angle', 0.1)  # radians

        # State tracking
        self.tasks: Dict[str, TaskAssignment] = {}
        self.robot_states: Dict[str, RobotState] = {}
        self.reference_points: Dict[str, ReferencePoint] = {}
        self.robot_positions: Dict[str, tuple] = {}  # (x, y, yaw)
        self.robot_current_task: Dict[str, Optional[str]] = {}

        # Publishers
        self.task_status_pub = self.create_publisher(String, '/task/status', 10)
        self.robot_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(Point, '/target_position', 10)
        self.robot_state_pub = self.create_publisher(String, '/robot/state', 10)

        # Subscribers
        self.task_assignment_sub = self.create_subscription(
            String,
            '/task/assignment',
            self.on_task_assignment,
            10
        )
        
        self.robot_pose_sub = self.create_subscription(
            Float32MultiArray,
            '/robot/pose',
            self.on_robot_pose_update,
            10
        )

        self.reference_point_sub = self.create_subscription(
            String,
            '/reference_point/update',
            self.on_reference_point_update,
            10
        )

        # Timer for main control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('Task Navigator Node initialized successfully')

    def on_task_assignment(self, msg: String) -> None:
        """
        Receive task assignment from backend
        Expected message format:
        {
            "task_id": "task_123",
            "robot_id": "robot_1",
            "target_x": 5.2,
            "target_y": 3.8,
            "target_yaw": 0.0,
            "priority": 1
        }
        """
        try:
            data = json.loads(msg.data)
            task = TaskAssignment(
                task_id=data.get('task_id'),
                robot_id=data.get('robot_id'),
                target_x=float(data.get('target_x', 0)),
                target_y=float(data.get('target_y', 0)),
                target_yaw=float(data.get('target_yaw', 0)),
                priority=int(data.get('priority', 0)),
                timestamp=time.time()
            )
            
            self.tasks[task.task_id] = task
            self.robot_current_task[task.robot_id] = task.task_id
            self.robot_states[task.robot_id] = RobotState.MOVING_TO_TARGET
            
            self.get_logger().info(
                f'Task assigned: {task.task_id} -> {task.robot_id} '
                f'to ({task.target_x}, {task.target_y})'
            )
            
            # Publish target position for visualization
            target_msg = Point(x=task.target_x, y=task.target_y, z=0.0)
            self.target_pub.publish(target_msg)
            
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'Failed to parse task assignment: {e}')

    def on_robot_pose_update(self, msg: Float32MultiArray) -> None:
        """
        Receive robot pose update (x, y, yaw)
        Expected format: [x, y, yaw]
        """
        try:
            if len(msg.data) >= 3:
                robot_id = msg.layout.dim[0].label if msg.layout.dim else 'unknown'
                x, y, yaw = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])
                self.robot_positions[robot_id] = (x, y, yaw)
                
                self.get_logger().debug(f'Robot {robot_id} pose: ({x:.2f}, {y:.2f}, {yaw:.2f})')
                
        except (IndexError, ValueError) as e:
            self.get_logger().error(f'Failed to parse robot pose: {e}')

    def on_reference_point_update(self, msg: String) -> None:
        """
        Receive reference point (home/dock) update
        Expected message format:
        {
            "robot_id": "robot_1",
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0
        }
        """
        try:
            data = json.loads(msg.data)
            ref_point = ReferencePoint(
                robot_id=data.get('robot_id'),
                x=float(data.get('x', 0)),
                y=float(data.get('y', 0)),
                yaw=float(data.get('yaw', 0))
            )
            
            self.reference_points[ref_point.robot_id] = ref_point
            self.get_logger().info(
                f'Reference point updated for {ref_point.robot_id}: '
                f'({ref_point.x}, {ref_point.y})'
            )
            
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'Failed to parse reference point: {e}')

    def control_loop(self) -> None:
        """Main control loop - processes all active tasks and robot movements"""
        try:
            # Process all active tasks
            for task_id, task in list(self.tasks.items()):
                robot_id = task.robot_id
                
                if robot_id not in self.robot_positions:
                    continue
                
                current_x, current_y, current_yaw = self.robot_positions[robot_id]
                state = self.robot_states.get(robot_id, RobotState.IDLE)

                # State machine for robot navigation
                if state == RobotState.MOVING_TO_TARGET:
                    # Calculate movement towards target
                    distance = math.hypot(
                        task.target_x - current_x,
                        task.target_y - current_y
                    )
                    
                    tolerance = self.get_parameter('tolerance_distance').value
                    
                    if distance < tolerance:
                        # Reached target
                        self.robot_states[robot_id] = RobotState.AT_TARGET
                        self.get_logger().info(
                            f'Robot {robot_id} reached target location '
                            f'({task.target_x}, {task.target_y})'
                        )
                        
                        # Publish task status
                        status_msg = String(
                            data=json.dumps({
                                'task_id': task_id,
                                'robot_id': robot_id,
                                'status': 'COMPLETED',
                                'timestamp': time.time()
                            })
                        )
                        self.task_status_pub.publish(status_msg)
                        
                        # Start return to reference point
                        if robot_id in self.reference_points:
                            self.robot_states[robot_id] = RobotState.RETURNING_TO_HOME
                    else:
                        # Send movement command
                        self.send_movement_command(
                            robot_id, task.target_x, task.target_y, task.target_yaw
                        )

                elif state == RobotState.RETURNING_TO_HOME:
                    # Return to reference point
                    if robot_id in self.reference_points:
                        ref_point = self.reference_points[robot_id]
                        
                        distance = math.hypot(
                            ref_point.x - current_x,
                            ref_point.y - current_y
                        )
                        
                        tolerance = self.get_parameter('tolerance_distance').value
                        
                        if distance < tolerance:
                            # Reached home
                            self.robot_states[robot_id] = RobotState.AT_HOME
                            self.get_logger().info(
                                f'Robot {robot_id} returned to reference point '
                                f'({ref_point.x}, {ref_point.y})'
                            )
                            
                            # Task completed - remove it
                            if task_id in self.tasks:
                                del self.tasks[task_id]
                            self.robot_current_task[robot_id] = None
                            self.robot_states[robot_id] = RobotState.IDLE
                        else:
                            # Send return command
                            self.send_movement_command(
                                robot_id, ref_point.x, ref_point.y, ref_point.yaw
                            )

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')

    def send_movement_command(
        self, robot_id: str, target_x: float, target_y: float, target_yaw: float
    ) -> None:
        """Send velocity command to move robot towards target"""
        try:
            current_x, current_y, current_yaw = self.robot_positions.get(
                robot_id, (0, 0, 0)
            )
            
            # Calculate desired velocity
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx, dy)
            
            # Desired angle towards target
            desired_angle = math.atan2(dy, dx)
            angle_diff = desired_angle - current_yaw
            
            # Normalize angle difference to [-pi, pi]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # Calculate velocities
            max_speed = self.get_parameter('robot_max_speed').value
            linear_velocity = min(max_speed, distance)
            angular_velocity = angle_diff * 2  # Simple proportional control
            
            # Create and publish velocity command
            cmd = Twist()
            cmd.linear.x = linear_velocity
            cmd.angular.z = angular_velocity
            
            self.robot_cmd_pub.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error sending movement command: {e}')

    def get_task_status(self, task_id: str) -> Optional[Dict]:
        """Get current status of a task"""
        if task_id not in self.tasks:
            return None
        
        task = self.tasks[task_id]
        robot_id = task.robot_id
        
        return {
            'task_id': task_id,
            'robot_id': robot_id,
            'status': self.robot_states.get(robot_id, RobotState.IDLE).value,
            'target_x': task.target_x,
            'target_y': task.target_y,
            'current_x': self.robot_positions.get(robot_id, (0, 0, 0))[0],
            'current_y': self.robot_positions.get(robot_id, (0, 0, 0))[1],
            'timestamp': time.time()
        }


def main(args=None):
    rclpy.init(args=args)
    node = TaskNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
