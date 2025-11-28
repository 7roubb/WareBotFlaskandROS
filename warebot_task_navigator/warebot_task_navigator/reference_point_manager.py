"""
Reference Point Manager - Manages home/dock locations for robots
Handles reference point configuration and updates
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import logging
from typing import Dict


@dataclass
class ReferencePoint:
    """Reference point data"""
    robot_id: str
    x: float
    y: float
    yaw: float
    description: str = ""


class ReferencePointManager(Node):
    """
    Manages reference points (home/dock locations) for robots
    - Stores reference point configurations
    - Publishes reference points to other nodes
    - Handles reference point updates from backend
    """

    def __init__(self):
        super().__init__('reference_point_manager')
        
        self.logger = logging.getLogger('ReferencePointManager')
        self.get_logger().info('Initializing Reference Point Manager')

        # Storage for reference points
        self.reference_points: Dict[str, dict] = {}
        
        # Load default reference points
        self.load_default_reference_points()

        # Publisher
        self.ref_point_pub = self.create_publisher(String, '/reference_point/update', 10)

        # Subscriber for updates from backend
        self.ref_point_update_sub = self.create_subscription(
            String,
            '/reference_point/command',
            self.on_reference_point_command,
            10
        )

        # Service (optional, for querying reference points)
        self.timer = self.create_timer(1.0, self.publish_reference_points)

        self.get_logger().info('Reference Point Manager initialized')

    def load_default_reference_points(self) -> None:
        """Load default reference points for all robots"""
        default_points = {
            'robot_1': {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'description': 'Dock A'},
            'robot_2': {'x': 2.0, 'y': 0.0, 'yaw': 0.0, 'description': 'Dock B'},
            'robot_3': {'x': 4.0, 'y': 0.0, 'yaw': 0.0, 'description': 'Dock C'},
        }
        
        self.reference_points = default_points
        self.get_logger().info(f'Loaded {len(default_points)} default reference points')

    def on_reference_point_command(self, msg: String) -> None:
        """
        Receive reference point update command from backend
        Expected format:
        {
            "command": "set" | "update",
            "robot_id": "robot_1",
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "description": "Main Dock"
        }
        """
        try:
            data = json.loads(msg.data)
            command = data.get('command', 'set')
            robot_id = data.get('robot_id')
            
            if not robot_id:
                self.get_logger().warn('Reference point command missing robot_id')
                return
            
            if command == 'set' or command == 'update':
                ref_point = {
                    'x': float(data.get('x', 0)),
                    'y': float(data.get('y', 0)),
                    'yaw': float(data.get('yaw', 0)),
                    'description': data.get('description', f'Reference point for {robot_id}')
                }
                
                self.reference_points[robot_id] = ref_point
                self.get_logger().info(
                    f'Reference point updated for {robot_id}: '
                    f'({ref_point["x"]}, {ref_point["y"]})'
                )
                
                # Publish immediately
                self.publish_reference_point(robot_id)
                
            elif command == 'delete':
                if robot_id in self.reference_points:
                    del self.reference_points[robot_id]
                    self.get_logger().info(f'Reference point deleted for {robot_id}')
            
            elif command == 'get':
                if robot_id in self.reference_points:
                    ref_point = self.reference_points[robot_id]
                    self.get_logger().info(
                        f'Reference point for {robot_id}: ({ref_point["x"]}, {ref_point["y"]})'
                    )
                else:
                    self.get_logger().warn(f'No reference point found for {robot_id}')
                    
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f'Failed to parse reference point command: {e}')

    def publish_reference_point(self, robot_id: str) -> None:
        """Publish a specific reference point"""
        if robot_id in self.reference_points:
            ref_point = self.reference_points[robot_id]
            
            msg = String(
                data=json.dumps({
                    'robot_id': robot_id,
                    'x': ref_point['x'],
                    'y': ref_point['y'],
                    'yaw': ref_point['yaw'],
                    'description': ref_point['description']
                })
            )
            self.ref_point_pub.publish(msg)

    def publish_reference_points(self) -> None:
        """Periodically publish all reference points"""
        for robot_id in self.reference_points:
            self.publish_reference_point(robot_id)

    def get_reference_point(self, robot_id: str) -> Dict:
        """Get reference point for a specific robot"""
        return self.reference_points.get(robot_id, {})

    def list_all_reference_points(self) -> Dict:
        """Get all reference points"""
        return self.reference_points


def main(args=None):
    rclpy.init(args=args)
    node = ReferencePointManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Need to add dataclass import
    from dataclasses import dataclass
    main()
