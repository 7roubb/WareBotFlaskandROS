#!/usr/bin/env python3
"""
WareBot Task Runner with Integrated AprilTag Alignment + Linear Actuator Control + Footprint Switching
Main entry point
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
from warebot_task_runner.robot_task_runner import RobotTaskRunner


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