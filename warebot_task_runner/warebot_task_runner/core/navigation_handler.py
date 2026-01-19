"""
Navigation Handler for Nav2 integration
"""
import math
from typing import Optional, Dict, Any, Callable
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NavigationHandler:
    """Handles Nav2 navigation"""
    
    def __init__(self, node, logger):
        self.node = node
        self.logger = logger
        
        # Nav2 action client
        self._nav_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.logger.info("Waiting for Nav2 server...")
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.logger.warn("Nav2 server not available yet. Still continuing.")
        else:
            self.logger.info("Nav2 server ready")
    
    def navigate_to(self, x: float, y: float, yaw: float, 
                    done_callback: Callable) -> bool:
        """
        Navigate to a pose
        
        Args:
            x: Target x position
            y: Target y position
            yaw: Target yaw orientation
            done_callback: Callback when goal is sent
            
        Returns:
            True if navigation started successfully
        """
        goal = self._create_nav_goal(x, y, yaw)
        
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.logger.warn("Nav2 not ready")
            return False
        
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(done_callback)
        return True
    
    def _create_nav_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        """Create navigation goal"""
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qz = math.sin(float(yaw) / 2.0)
        qw = math.cos(float(yaw) / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose
        return goal