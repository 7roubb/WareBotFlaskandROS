#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
import os

from pupil_apriltags import Detector


# =============================
# PID Helper
# =============================
class PID:
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
    if abs(cmd) < 1e-6:
        return 0.0
    return math.copysign(max(min_mag, abs(cmd)), cmd)


# =============================
# Node
# =============================
class AprilTagAlignUpward(Node):
    def __init__(self):
        super().__init__('apriltag_align_upward')

        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('image_topic', '/usb_cam/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('estop_topic', '/emergency_stop_state')
        self.declare_parameter('alignment_status_topic', '/apriltag/alignment_status')
        self.declare_parameter('show_debug_image', True)
        self.declare_parameter('lost_tag_timeout', 0.7)
        self.declare_parameter('camera_matrix_path', '')
        self.declare_parameter('dist_coeffs_path', '')

        # Read parameters
        img_topic = self.get_parameter('image_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        estop_topic = self.get_parameter('estop_topic').value
        status_topic = self.get_parameter('alignment_status_topic').value
        self.show_debug = self.get_parameter('show_debug_image').value
        self.lost_tag_timeout = self.get_parameter('lost_tag_timeout').value
        cam_matrix_path = self.get_parameter('camera_matrix_path').value
        dist_coeffs_path = self.get_parameter('dist_coeffs_path').value

        # Load calibration
        if not os.path.exists(cam_matrix_path):
            raise FileNotFoundError(f"Camera matrix not found: {cam_matrix_path}")
        if not os.path.exists(dist_coeffs_path):
            raise FileNotFoundError(f"Distortion coeffs not found: {dist_coeffs_path}")
        self.camera_matrix = np.load(cam_matrix_path)
        self.dist_coeffs = np.load(dist_coeffs_path)

        # AprilTag detector
        self.detector = Detector()

        # Tag size
        self.TAG_SIZE = 0.16

        # High-precision tolerances
        self.TOLERANCE_X = 0.03
        self.TOLERANCE_Z = 0.03
        self.TOLERANCE_YAW = 4.0

        self.obj_pts = np.array([
            [-self.TAG_SIZE/2,  self.TAG_SIZE/2, 0],
            [ self.TAG_SIZE/2,  self.TAG_SIZE/2, 0],
            [ self.TAG_SIZE/2, -self.TAG_SIZE/2, 0],
            [-self.TAG_SIZE/2, -self.TAG_SIZE/2, 0],
        ], dtype=np.float32)

        # PID Controllers
        self.pid_yaw     = PID(0.06, 0.0, 0.020, i_limit=0.5, out_limit=0.35)
        self.pid_strafe  = PID(1.2, 0.0, 0.25, i_limit=0.4, out_limit=0.15)
        self.pid_forward = PID(1.2, 0.0, 0.25, i_limit=0.4, out_limit=0.10)

        # Minimum speeds
        self.min_wz = 0.05
        self.min_vy = 0.02
        self.min_vx = 0.02

        # Alignment control
        self.alignment_enabled = False
        self.alignment_complete = False

        # ROS interfaces
        self.img_sub = self.create_subscription(Image, img_topic, self.image_cb, 10)
        self.estop_sub = self.create_subscription(Bool, estop_topic, self.estop_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        
        # Service to enable/disable alignment
        self.alignment_service = self.create_service(
            SetBool, 
            '/apriltag/enable_alignment', 
            self.enable_alignment_callback
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        # Filtered values
        self.sx = None
        self.sz = None
        self.syaw = None
        self.tag_found = False
        self.last_tag_time = None
        self.x = 0.0
        self.z = 0.0
        self.yaw_deg = 0.0
        self.estop_active = False

        self.get_logger().info("=" * 50)
        self.get_logger().info("UPWARD APRILTAG ALIGN NODE STARTED")
        self.get_logger().info("Waiting for alignment enable command...")
        self.get_logger().info("=" * 50)

    # ---------------------------------------------
    def enable_alignment_callback(self, request, response):
        """Service callback to enable/disable alignment"""
        self.alignment_enabled = request.data
        
        if self.alignment_enabled:
            self.get_logger().info("🎯 Alignment ENABLED - Starting alignment process")
            self.alignment_complete = False
            # Reset PIDs
            self.pid_yaw.reset()
            self.pid_strafe.reset()
            self.pid_forward.reset()
            # Reset filtered values
            self.sx = None
            self.sz = None
            self.syaw = None
        else:
            self.get_logger().info("⏸️  Alignment DISABLED")
            self.alignment_complete = False
            # Stop the robot
            self.cmd_pub.publish(Twist())
        
        response.success = True
        response.message = f"Alignment {'enabled' if request.data else 'disabled'}"
        return response

    # ---------------------------------------------
    def estop_cb(self, msg):
        self.estop_active = msg.data
        if msg.data:
            self.pid_yaw.reset()
            self.pid_strafe.reset()
            self.pid_forward.reset()

    # ---------------------------------------------
    def image_cb(self, msg):
        if not self.alignment_enabled:
            return  # Don't process images if alignment is disabled
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        if len(detections) == 0:
            self.tag_found = False
            return  # Keep last x, z, yaw

        det = detections[0]
        corners = np.array(det.corners, dtype=np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            self.obj_pts, corners, self.camera_matrix, self.dist_coeffs
        )
        if not ok:
            self.tag_found = False
            return

        x = float(tvec[0])
        z = float(tvec[1])

        R, _ = cv2.Rodrigues(rvec)
        yaw_rad = math.atan2(-R[1, 0], R[0, 0])
        yaw_deg = float(np.degrees(yaw_rad))
        if yaw_deg > 180: yaw_deg -= 360
        if yaw_deg < -180: yaw_deg += 360

        # Smooth filter
        alpha = 0.30
        if self.sx is None:
            self.sx, self.sz, self.syaw = x, z, yaw_deg
        else:
            self.sx = self.sx * (1-alpha) + x * alpha
            self.sz = self.sz * (1-alpha) + z * alpha
            self.syaw = self.syaw * (1-alpha) + yaw_deg * alpha

        self.x = self.sx
        self.z = self.sz
        self.yaw_deg = self.syaw
        self.tag_found = True
        self.last_tag_time = time.time()

        if self.show_debug:
            txt = f"X:{self.x:+.3f} Z:{self.z:+.3f} Yaw:{self.yaw_deg:+.1f}"
            status_txt = "ALIGNING" if self.alignment_enabled else "DISABLED"
            cv2.putText(frame, txt, (10,40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
            cv2.putText(frame, status_txt, (10,80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if self.alignment_enabled else (0,0,255), 2)
            cv2.imshow("AprilTag-Upward", frame)
            cv2.waitKey(1)

    # ---------------------------------------------
    def control_loop(self):
        twist = Twist()

        # Don't control if alignment is disabled or estop is active
        if not self.alignment_enabled or self.estop_active:
            self.cmd_pub.publish(twist)
            return

        ex = self.x
        ez = self.z
        yaw = self.yaw_deg

        x_aligned = abs(ex) <= self.TOLERANCE_X
        z_aligned = abs(ez) <= self.TOLERANCE_Z
        yaw_aligned = abs(yaw) <= self.TOLERANCE_YAW

        # Check if perfectly aligned
        if x_aligned and z_aligned and yaw_aligned:
            if not self.alignment_complete:
                self.alignment_complete = True
                self.get_logger().info("🎯 PERFECT ALIGNMENT ACHIEVED!")
                # Publish status
                status_msg = String()
                status_msg.data = "ALIGNED"
                self.status_pub.publish(status_msg)
            # Stop the robot
            self.cmd_pub.publish(twist)
            return

        # Continue aligning
        wz = self.pid_yaw.step(-yaw)
        vy = self.pid_strafe.step(-ex)
        vx = self.pid_forward.step(-ez)

        if yaw_aligned: wz = 0
        if x_aligned: vy = 0
        if z_aligned: vx = 0

        if not yaw_aligned: wz = apply_min(wz, self.min_wz)
        if not x_aligned: vy = apply_min(vy, self.min_vy)
        if not z_aligned: vx = apply_min(vx, self.min_vx)

        twist.angular.z = float(np.clip(wz, -0.35, 0.35))
        twist.linear.y  = float(np.clip(vy, -0.15, 0.15))
        twist.linear.x  = float(np.clip(vx, -0.10, 0.10))

        self.cmd_pub.publish(twist)
        
        # Publish aligning status
        status_msg = String()
        status_msg.data = "ALIGNING"
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagAlignUpward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()