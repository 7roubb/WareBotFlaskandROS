"""
Task Executor Node - Executes movement commands on robots
Receives velocity commands and sends them to the robot hardware
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import json
import logging
import math
import time


class TaskExecutor(Node):
    """
    Node that executes movement commands on robots
    - Receives velocity commands from TaskNavigator
    - Sends commands to robot hardware via MQTT or direct interface
    - Monitors robot odometry and publishes pose updates
    """

    def __init__(self):
        super().__init__('task_executor')
        
        self.logger = logging.getLogger('TaskExecutor')
        self.get_logger().info('Initializing Task Executor Node')

        # Configuration
        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)

        self.robot_id = self.get_parameter('robot_id').value

        # State
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.current_velocity = {'linear_x': 0.0, 'angular_z': 0.0}

        # Publishers
        self.odometry_pub = self.create_publisher(Float32MultiArray, '/robot/pose', 10)
        self.movement_status_pub = self.create_publisher(String, '/movement/status', 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.on_cmd_vel,
            10
        )

        self.odometry_sub = self.create_subscription(
            Float32MultiArray,
            '/odometry',
            self.on_odometry,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.execute_command)

        self.get_logger().info(f'Task Executor initialized for {self.robot_id}')

    def on_cmd_vel(self, msg: Twist) -> None:
        """Receive velocity command"""
        self.current_velocity['linear_x'] = msg.linear.x
        self.current_velocity['angular_z'] = msg.angular.z
        
        self.get_logger().debug(
            f'Velocity command: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}'
        )

    def on_odometry(self, msg: Float32MultiArray) -> None:
        """Receive odometry update from robot hardware"""
        try:
            if len(msg.data) >= 3:
                self.current_pose['x'] = float(msg.data[0])
                self.current_pose['y'] = float(msg.data[1])
                self.current_pose['yaw'] = float(msg.data[2])
                
                self.get_logger().debug(
                    f'Odometry update: ({self.current_pose["x"]:.2f}, '
                    f'{self.current_pose["y"]:.2f}, {self.current_pose["yaw"]:.2f})'
                )
                
        except (IndexError, ValueError) as e:
            self.get_logger().error(f'Failed to parse odometry: {e}')

    def execute_command(self) -> None:
        """Execute the current velocity command on the robot"""
        try:
            # Simulate robot movement (in real scenario, this would send to hardware)
            linear_velocity = self.current_velocity['linear_x']
            angular_velocity = self.current_velocity['angular_z']

            # Update pose based on velocity (simple integration)
            dt = 0.1  # 100ms time step
            
            if linear_velocity > 0.001 or abs(angular_velocity) > 0.001:
                # Update position
                self.current_pose['yaw'] += angular_velocity * dt
                self.current_pose['x'] += linear_velocity * math.cos(self.current_pose['yaw']) * dt
                self.current_pose['y'] += linear_velocity * math.sin(self.current_pose['yaw']) * dt
                
                # Normalize yaw
                self.current_pose['yaw'] = math.atan2(
                    math.sin(self.current_pose['yaw']),
                    math.cos(self.current_pose['yaw'])
                )
                
                # Publish updated pose
                self.publish_pose()
                
                # Send to robot hardware (MQTT or direct command)
                self.send_to_hardware()

        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')

    def publish_pose(self) -> None:
        """Publish current robot pose"""
        msg = Float32MultiArray()
        msg.data = [
            self.current_pose['x'],
            self.current_pose['y'],
            self.current_pose['yaw']
        ]
        self.odometry_pub.publish(msg)

    def send_to_hardware(self) -> None:
        """Send velocity command to robot hardware"""
        try:
            # In a real implementation, this would:
            # 1. Send via MQTT to the robot
            # 2. Or directly control motor controllers
            # 3. Or send to ROS2 hardware interface
            
            status_msg = String(
                data=json.dumps({
                    'robot_id': self.robot_id,
                    'status': 'executing',
                    'linear_velocity': self.current_velocity['linear_x'],
                    'angular_velocity': self.current_velocity['angular_z'],
                    'pose_x': self.current_pose['x'],
                    'pose_y': self.current_pose['y'],
                    'pose_yaw': self.current_pose['yaw'],
                    'timestamp': time.time()
                })
            )
            self.movement_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error sending to hardware: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TaskExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
