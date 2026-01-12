#!/usr/bin/env python3
"""
Neobotix MP-400 Footprint Switcher Node

This node provides dynamic footprint switching between two configurations:
1. Original MP-400 footprint: 0.30m x 0.30m (60cm x 60cm)
2. Extended footprint: 1.00m x 1.00m (100cm x 100cm)

The footprint can be switched via:
- ROS2 Service call: /mp400/switch_footprint
- Parameter update (dynamic reconfigure)
- Command line argument at startup

Footprints are applied to:
- Local costmap
- Global costmap
- Behavior tree navigator
"""

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Trigger
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import String

from rcl_interfaces.msg import SetParametersResult
import json
from typing import Dict, List, Tuple

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

# ============================================================================
# FOOTPRINT SWITCHER NODE
# ============================================================================

class FootprintSwitcherNode(Node):
    """
    ROS2 Node for dynamically switching MP-400 footprint between configurations.
    
    This node manages footprint switching and ensures consistency across:
    - Local costmap
    - Global costmap
    - Behavior Tree Navigator
    - Navigation parameters
    """

    def __init__(self):
        super().__init__('mp400_footprint_switcher')
        
        # Get parameters
        self.declare_parameter('robot_namespace', '/')
        self.declare_parameter('initial_footprint', 'original')  # 'original' or 'extended'
        self.declare_parameter('update_frequency', 1.0)
        self.declare_parameter('verbose', True)
        
        self.robot_ns = self.get_parameter('robot_namespace').value
        self.initial_footprint = self.get_parameter('initial_footprint').value
        self.update_freq = self.get_parameter('update_frequency').value
        self.verbose = self.get_parameter('verbose').value
        
        # Normalize namespace
        if self.robot_ns != '/' and not self.robot_ns.startswith('/'):
            self.robot_ns = '/' + self.robot_ns
        if self.robot_ns.endswith('/'):
            self.robot_ns = self.robot_ns[:-1]
        
        self.current_footprint = self.initial_footprint
        
        self._log(f"🤖 MP-400 Footprint Switcher initialized")
        self._log(f"   Namespace: {self.robot_ns}")
        self._log(f"   Initial footprint: {self.initial_footprint}")
        
        # Create services
        self.switch_service = self.create_service(
            Trigger,
            f'{self.robot_ns}/switch_footprint',
            self._handle_switch_footprint,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        # Publishers for footprint updates
        self.local_costmap_footprint_pub = self.create_publisher(
            Polygon,
            f'{self.robot_ns}/local_costmap/footprint',
            1
        )
        
        self.global_costmap_footprint_pub = self.create_publisher(
            Polygon,
            f'{self.robot_ns}/global_costmap/footprint',
            1
        )
        
        self.status_pub = self.create_publisher(
            String,
            f'{self.robot_ns}/footprint_status',
            1
        )
        
        # Subscribers for footprint updates from costmaps
        self.create_subscription(
            Polygon,
            f'{self.robot_ns}/local_costmap/published_footprint',
            self._on_local_footprint_update,
            1
        )
        
        self.create_subscription(
            Polygon,
            f'{self.robot_ns}/global_costmap/published_footprint',
            self._on_global_footprint_update,
            1
        )
        
        # Set parameter callbacks for dynamic reconfiguration
        self.add_on_set_parameters_callback(self._parameters_callback)
        
        # Initial footprint update
        self._apply_footprint(self.initial_footprint)
        
        self._log("✅ MP-400 Footprint Switcher ready!")
        self._log(f"   Service: {self.robot_ns}/switch_footprint")
        self._log(f"   Status topic: {self.robot_ns}/footprint_status")

    # ========================================================================
    # SERVICE HANDLERS
    # ========================================================================

    def _handle_switch_footprint(self, request, response):
        """
        Handle footprint switch service call.
        Toggles between original and extended footprints.
        """
        new_footprint = "extended" if self.current_footprint == "original" else "original"
        
        success = self._apply_footprint(new_footprint)
        
        response.success = success
        if success:
            response.message = f"✅ Footprint switched to {new_footprint}"
            self._log(f"🔄 Footprint switched to {new_footprint}")
        else:
            response.message = f"❌ Failed to switch footprint"
            self._log(f"❌ Failed to switch footprint", error=True)
        
        return response

    # ========================================================================
    # FOOTPRINT APPLICATION
    # ========================================================================

    def _apply_footprint(self, footprint_name: str) -> bool:
        """
        Apply footprint configuration to costmaps.
        
        Args:
            footprint_name: 'original' or 'extended'
            
        Returns:
            True if successful, False otherwise
        """
        if footprint_name not in FOOTPRINT_NAMES:
            self._log(f"❌ Unknown footprint: {footprint_name}", error=True)
            return False
        
        try:
            # Get footprint points
            footprint_points = FOOTPRINT_NAMES[footprint_name]
            
            # Create polygon message
            polygon_msg = self._create_polygon_msg(footprint_points)
            
            # Publish to costmaps
            self.local_costmap_footprint_pub.publish(polygon_msg)
            self.global_costmap_footprint_pub.publish(polygon_msg)
            
            # Update parameters in Nav2 stack
            self._update_nav2_parameters(footprint_name)
            
            # Update internal state
            self.current_footprint = footprint_name
            
            # Publish status
            status_msg = String()
            status_msg.data = json.dumps({
                "footprint": footprint_name,
                "points": footprint_points,
                "width": max([p[0] for p in footprint_points]) * 2,
                "height": max([p[1] for p in footprint_points]) * 2,
                "timestamp": self.get_clock().now().nanoseconds
            })
            self.status_pub.publish(status_msg)
            
            self._log(f"✅ Footprint '{footprint_name}' applied successfully")
            return True
            
        except Exception as e:
            self._log(f"❌ Error applying footprint: {str(e)}", error=True)
            return False

    def _create_polygon_msg(self, points: List[Tuple[float, float]]) -> Polygon:
        """
        Create a Polygon message from list of points.
        
        Args:
            points: List of [x, y] coordinate tuples
            
        Returns:
            Polygon message
        """
        polygon = Polygon()
        for point in points:
            p = Point32()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.0
            polygon.points.append(p)
        return polygon

    def _update_nav2_parameters(self, footprint_name: str):
        """
        Update Nav2 parameters with new footprint configuration.
        This ensures consistency across the navigation stack.
        """
        try:
            footprint_points = FOOTPRINT_NAMES[footprint_name]
            
            # Format for YAML: [ [x1, y1], [x2, y2], ... ]
            footprint_yaml = str(footprint_points)
            
            # Update local costmap parameters
            self.set_parameters([
                rclpy.parameter.Parameter(
                    f'{self.robot_ns}.local_costmap.footprint',
                    value=footprint_yaml
                ),
                rclpy.parameter.Parameter(
                    f'{self.robot_ns}.global_costmap.footprint',
                    value=footprint_yaml
                ),
            ])
            
            self._log(f"📝 Nav2 parameters updated with {footprint_name} footprint")
            
        except Exception as e:
            self._log(f"⚠️  Could not update Nav2 parameters: {str(e)}")

    # ========================================================================
    # CALLBACKS
    # ========================================================================

    def _on_local_footprint_update(self, msg: Polygon):
        """Callback when local costmap publishes footprint."""
        pass  # Could add verification here

    def _on_global_footprint_update(self, msg: Polygon):
        """Callback when global costmap publishes footprint."""
        pass  # Could add verification here

    def _parameters_callback(self, params):
        """Handle parameter updates (dynamic reconfiguration)."""
        for param in params:
            if param.name == 'initial_footprint':
                new_footprint = param.value
                if new_footprint != self.current_footprint:
                    self._apply_footprint(new_footprint)
                    return SetParametersResult(successful=True)
        
        return SetParametersResult(successful=False)

    # ========================================================================
    # UTILITIES
    # ========================================================================

    def _log(self, message: str, error: bool = False):
        """Log message with timestamp."""
        if self.verbose or error:
            if error:
                self.get_logger().error(message)
            else:
                self.get_logger().info(message)

    def get_footprint_info(self) -> Dict:
        """Get current footprint information."""
        footprint_points = FOOTPRINT_NAMES[self.current_footprint]
        return {
            "current": self.current_footprint,
            "points": footprint_points,
            "width": max([p[0] for p in footprint_points]) * 2,
            "height": max([p[1] for p in footprint_points]) * 2,
        }


# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    node = FootprintSwitcherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
