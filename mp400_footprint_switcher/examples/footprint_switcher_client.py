#!/usr/bin/env python3
"""
Example client for MP-400 Footprint Switcher

This script demonstrates how to interact with the footprint switcher node
to dynamically change the robot's footprint configuration.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import json
import time
import sys


class FootprintSwitcherClient(Node):
    """Client for controlling MP-400 footprint switching."""
    
    def __init__(self, robot_namespace=''):
        super().__init__('footprint_switcher_client')
        
        # Setup namespace
        if robot_namespace and not robot_namespace.startswith('/'):
            robot_namespace = '/' + robot_namespace
        if robot_namespace and robot_namespace.endswith('/'):
            robot_namespace = robot_namespace[:-1]
        
        self.namespace = robot_namespace
        self.current_footprint = None
        
        # Create service client
        service_name = f'{self.namespace}/switch_footprint' if self.namespace else '/mp400/switch_footprint'
        self.switch_client = self.create_client(Trigger, service_name)
        
        # Subscribe to status updates
        status_topic = f'{self.namespace}/footprint_status' if self.namespace else '/mp400/footprint_status'
        self.status_sub = self.create_subscription(
            String,
            status_topic,
            self._on_status_update,
            1
        )
        
        # Wait for service
        while not self.switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'⏳ Waiting for service: {service_name}')
        
        self.get_logger().info(f'✅ Connected to {service_name}')
    
    def _on_status_update(self, msg: String):
        """Handle status updates from the switcher node."""
        try:
            status = json.loads(msg.data)
            self.current_footprint = status.get('footprint')
            self.get_logger().debug(f'📊 Status update: {status}')
        except json.JSONDecodeError:
            self.get_logger().warning(f'Failed to parse status: {msg.data}')
    
    def switch_footprint(self) -> bool:
        """
        Switch footprint to the alternate configuration.
        
        Returns:
            True if successful, False otherwise
        """
        self.get_logger().info('📤 Sending switch_footprint request...')
        
        request = Trigger.Request()
        future = self.switch_client.call_async(request)
        
        # Wait for response with timeout
        timeout = 5.0
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error('⏱️  Service call timed out')
                return False
            time.sleep(0.1)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ {response.message}')
                return True
            else:
                self.get_logger().error(f'❌ {response.message}')
                return False
        except Exception as e:
            self.get_logger().error(f'Exception in service call: {e}')
            return False
    
    def get_current_footprint(self) -> str:
        """Get the currently active footprint configuration."""
        return self.current_footprint if self.current_footprint else "Unknown"
    
    def switch_to_original(self) -> bool:
        """Switch to original footprint (if not already)."""
        if self.get_current_footprint() == 'original':
            self.get_logger().info("ℹ️  Already using original footprint")
            return True
        return self.switch_footprint()
    
    def switch_to_extended(self) -> bool:
        """Switch to extended footprint (if not already)."""
        if self.get_current_footprint() == 'extended':
            self.get_logger().info("ℹ️  Already using extended footprint")
            return True
        return self.switch_footprint()


def main(args=None):
    """Main function with command-line interface."""
    rclpy.init(args=args)
    
    # Parse command line arguments
    robot_namespace = ''
    action = 'toggle'
    
    if len(sys.argv) > 1:
        robot_namespace = sys.argv[1]
    if len(sys.argv) > 2:
        action = sys.argv[2].lower()
    
    # Create client
    client = FootprintSwitcherClient(robot_namespace)
    
    try:
        # Give some time for initial status update
        time.sleep(0.5)
        
        print("\n" + "="*50)
        print("  MP-400 Footprint Switcher Client")
        print("="*50)
        print(f"📍 Robot Namespace: {client.namespace if client.namespace else 'default'}")
        print(f"📊 Current Footprint: {client.get_current_footprint()}")
        print("="*50 + "\n")
        
        # Execute requested action
        if action == 'toggle':
            print("🔄 Toggling footprint...")
            success = client.switch_footprint()
        elif action == 'original':
            print("🔄 Switching to original footprint...")
            success = client.switch_to_original()
        elif action == 'extended':
            print("🔄 Switching to extended footprint...")
            success = client.switch_to_extended()
        elif action == 'status':
            print(f"📊 Current footprint: {client.get_current_footprint()}")
            success = True
        else:
            print(f"❌ Unknown action: {action}")
            print_usage()
            success = False
        
        if success:
            time.sleep(1)  # Wait for status update
            print(f"\n✅ Current footprint: {client.get_current_footprint()}\n")
        else:
            print("\n❌ Operation failed\n")
        
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}\n")
    finally:
        client.destroy_node()
        rclpy.shutdown()


def print_usage():
    """Print usage information."""
    print("\nUsage:")
    print("  python3 footprint_switcher_client.py [namespace] [action]")
    print("\nArguments:")
    print("  namespace - Robot namespace (optional, e.g., 'robot1')")
    print("  action    - Action to perform:")
    print("              toggle   - Toggle between original and extended (default)")
    print("              original - Switch to original footprint")
    print("              extended - Switch to extended footprint")
    print("              status   - Print current footprint")
    print("\nExamples:")
    print("  python3 footprint_switcher_client.py")
    print("  python3 footprint_switcher_client.py robot1 toggle")
    print("  python3 footprint_switcher_client.py robot1 original")
    print("  python3 footprint_switcher_client.py robot1 extended")
    print()


if __name__ == '__main__':
    main()
