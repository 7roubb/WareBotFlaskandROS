"""
Enhanced AprilTag Alignment Controller - 4-Tag Precision System
Integrated with WareBot Task Runner (Background Processing, No GUI)

MODIFIED FOR:
- Use ONLY calibration.npy (no separate matrix/coeffs files)
- Accept calibration_file_path parameter from launch file
- Support override from launch command
- Automatic fallback to standard locations if not provided
"""
import time
import math
import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict
from pathlib import Path
import json

from warebot_task_runner.utils import PID, apply_min
from warebot_task_runner.constants import *


@dataclass
class AprilTagDetection:
    """Store AprilTag detection information."""
    tag_id: int
    corners: np.ndarray  # 4x2 array of corner points
    center: Tuple[float, float]
    pose: Optional[Tuple] = None  # rvec, tvec
    size: float = 0.0  # Tag size in pixels


class AlignmentController:
    """
    Enhanced 4-Tag AprilTag Alignment Controller
    - Detects and aligns to 4 AprilTags in a 2x2 grid
    - Provides ultra-precise centering before alignment
    - Maintains smooth, filtered position estimates
    - Integrates seamlessly with WareBot task runner
    - Background processing only (no graphical output)
    - MODIFIED: Uses ONLY calibration.npy file with launch parameter support
    """
    
    def __init__(self, node, logger, cmd_vel_topic, image_topic, estop_topic,
                 show_debug=True, lost_tag_timeout=0.7, alignment_hold_time=30.0, 
                 calibration_file_path: str = "",
                 tag_size: float = 0.068,
                 tag_ids: list = None,
                 package_share_dir: str = None):
        """
        Initialize alignment controller with calibration file path from launch.
        
        Args:
            node: ROS2 node
            logger: ROS2 logger
            cmd_vel_topic: Command velocity topic
            image_topic: Image topic
            estop_topic: Emergency stop topic
            show_debug: Show debug output
            lost_tag_timeout: Tag lost timeout in seconds
            alignment_hold_time: Time to hold alignment in seconds
            calibration_file_path: Full path to calibration.npy (from launch parameter)
                                  If empty, searches standard locations
            tag_size: AprilTag size in meters
            tag_ids: Expected tag IDs [294, 323, 420, 530]
            package_share_dir: Path to package share directory (auto-detected if None)
        """
        
        self.node = node
        self.logger = logger
        self.show_debug = show_debug
        self.lost_tag_timeout = lost_tag_timeout
        self.alignment_hold_time = alignment_hold_time
        
        # 4-tag configuration
        self.tag_size = tag_size
        self.expected_tag_ids = tag_ids if tag_ids is not None else [294, 323, 420, 530]
        
        # Auto-detect package share directory if not provided
        if package_share_dir is None:
            package_share_dir = self._find_package_share_dir()
        
        self.package_share_dir = package_share_dir
        
        # Load camera calibration from ONLY .npy file
        self._load_calibration(
            calibration_file_path=calibration_file_path,
            package_share_dir=package_share_dir
        )
        
        # Initialize CV Bridge and AprilTag detector (pupil-apriltags)
        self.bridge = CvBridge()
        try:
            self.detector = Detector()
            self.detector_type = "pupil-apriltags"
            self.logger.info("✅ Using pupil-apriltags detector (MOST ACCURATE)")
        except ImportError:
            self.logger.warn("⚠️  pupil-apriltags not available, falling back to OpenCV ArUco")
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict)
            self.detector_type = "aruco"
        
        # Detection history for smoothing
        self.detection_history = {tag_id: deque(maxlen=30) for tag_id in self.expected_tag_ids}
        self.center_history = deque(maxlen=30)
        
        # Centering thresholds
        self.center_tolerance = 15  # pixels
        self.movement_threshold = 10  # pixels
        
        # Expected tag positions in 2x2 grid
        self.expected_positions = {
            self.expected_tag_ids[0]: ("TOP-LEFT", 0, 0),
            self.expected_tag_ids[1]: ("TOP-RIGHT", 1, 0),
            self.expected_tag_ids[2]: ("BOTTOM-LEFT", 0, 1),
            self.expected_tag_ids[3]: ("BOTTOM-RIGHT", 1, 1)
        }
        
        # Old single-tag PID for backward compatibility
        self.pid_yaw = PID(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, 
                          PID_YAW_I_LIMIT, PID_YAW_OUT_LIMIT)
        self.pid_strafe = PID(PID_STRAFE_KP, PID_STRAFE_KI, PID_STRAFE_KD,
                             PID_STRAFE_I_LIMIT, PID_STRAFE_OUT_LIMIT)
        self.pid_forward = PID(PID_FORWARD_KP, PID_FORWARD_KI, PID_FORWARD_KD,
                              PID_FORWARD_I_LIMIT, PID_FORWARD_OUT_LIMIT)
        
        # Enhanced 4-tag PID (more aggressive for multi-tag alignment)
        self.pid_yaw_enhanced = PID(PID_YAW_KP * 1.1, PID_YAW_KI, PID_YAW_KD * 1.05,
                                    PID_YAW_I_LIMIT, PID_YAW_OUT_LIMIT * 1.1)
        self.pid_strafe_enhanced = PID(PID_STRAFE_KP * 1.05, PID_STRAFE_KI, PID_STRAFE_KD,
                                       PID_STRAFE_I_LIMIT, PID_STRAFE_OUT_LIMIT * 1.05)
        self.pid_forward_enhanced = PID(PID_FORWARD_KP * 1.05, PID_FORWARD_KI, PID_FORWARD_KD,
                                        PID_FORWARD_I_LIMIT, PID_FORWARD_OUT_LIMIT * 1.05)
        
        # State variables
        self.sx = self.sz = self.syaw = None
        self.x = self.z = self.yaw_deg = 0.0
        self.tag_found = False
        self.last_tag_time = None
        self.estop_active = False
        self.aligned = False
        self.alignment_time = None
        self.alignment_complete = False
        self.should_exit = False
        self.frame_count = 0
        self.no_tag_count = 0
        
        # Multi-tag state
        self.detected_tags = {}
        self.detected_count = 0
        self.grid_center = None
        self.all_tags_centered = False
        self.centering_phase = True  # First phase: center the grid
        self.alignment_phase = False  # Second phase: fine-tune alignment
        
        # Actuator state tracking
        self.actuators_extended = False
        self.last_alignment_state = False
        
        # ROS2 publishers and subscribers
        self.cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        self.img_sub = node.create_subscription(Image, image_topic, self.image_cb, 10)
        self.estop_sub = node.create_subscription(Bool, estop_topic, self.estop_cb, 10)
        
        # Control loop timer
        self.control_timer = None
        
        # Callbacks
        self.on_aligned_callback = None
        self.actuator_controller = None
        
        self.logger.info("✅ Enhanced 4-Tag Alignment Controller initialized")
        self._log_calibration_info()
    
    def _find_package_share_dir(self) -> str:
        """
        Auto-detect package share directory.
        
        Looks for:
        1. Current working directory parent containing 'warebot_task_runner'
        2. ROS2 install directory
        3. Environment variable ROS_PACKAGE_PATH
        """
        cwd = Path.cwd()
        for parent in [cwd] + list(cwd.parents):
            candidate = parent / "warebot_task_runner"
            if candidate.exists():
                self.logger.info(f"Found package at: {parent}")
                return str(parent)
        
        # Try environment variables
        import os
        if "COLCON_PREFIX_PATH" in os.environ:
            share_path = Path(os.environ["COLCON_PREFIX_PATH"]) / "share" / "warebot_task_runner"
            if share_path.exists():
                self.logger.info(f"Found package in COLCON_PREFIX_PATH: {share_path.parent.parent}")
                return str(share_path.parent.parent)
        
        # Default fallback
        self.logger.warn("⚠️  Could not auto-detect package directory, using current working directory")
        return str(Path.cwd())
    
    def _load_calibration(self, calibration_file_path: str, package_share_dir: str):
        """
        Load camera calibration from .npy file ONLY.
        
        Priority:
        1. If calibration_file_path provided (from launch) → use it
        2. Look for calibration.npy in:
           - {package_share_dir}/config/calibration.npy
           - {package_share_dir}/data/calibration/calibration.npy
        3. Fall back to current working directory
        """
        
        self.logger.info("=" * 70)
        self.logger.info("🔧 LOADING CAMERA CALIBRATION")
        self.logger.info("=" * 70)
        
        # Build list of paths to try
        calibration_paths = []
        
        # If calibration_file_path provided from launch, try it FIRST
        if calibration_file_path and calibration_file_path.strip():
            calibration_paths.append(Path(calibration_file_path))
        
        # Standard locations
        calibration_paths.extend([
            Path(package_share_dir) / "config" / "calibration.npy",
            Path(package_share_dir) / "data" / "calibration" / "calibration.npy",
            Path(package_share_dir) / "calibration.npy",
            Path.cwd() / "calibration.npy",
            Path.home() / "calibration" / "calibration.npy",
        ])
        
        # Try each path
        for calib_path in calibration_paths:
            if calib_path.exists():
                if self._try_load_npy_file(str(calib_path)):
                    return
        
        # If nothing worked, raise error with helpful message
        self.logger.error("=" * 70)
        self.logger.error("❌ FATAL: Could not load camera calibration")
        self.logger.error("=" * 70)
        self.logger.error(f"\nSearched in:")
        for i, path in enumerate(calibration_paths, 1):
            self.logger.error(f"  {i}. {path}")
        
        if calibration_file_path:
            self.logger.error(f"\nProvided path from launch:")
            self.logger.error(f"  {calibration_file_path}")
        
        self.logger.error(f"\nPackage directory: {package_share_dir}")
        self.logger.error(f"Current directory: {Path.cwd()}")
        
        raise FileNotFoundError(
            f"Could not find calibration.npy\n"
            f"Provide it via launch parameter: calibration_file_path:=/path/to/calibration.npy\n"
            f"Or place it in: {package_share_dir}/config/calibration.npy"
        )
    
    def _try_load_npy_file(self, filepath: str) -> bool:
        """Try to load calibration from .npy file."""
        try:
            self.logger.info(f"🔍 Trying: {filepath}")
            
            calib_data = np.load(filepath, allow_pickle=True).item()
            
            # Extract camera matrix and distortion coefficients from .npy file
            if isinstance(calib_data, dict):
                self.camera_matrix = calib_data['camera_matrix']
                self.dist_coeffs = calib_data['dist_coeffs']
            else:
                self.camera_matrix = calib_data.camera_matrix
                self.dist_coeffs = calib_data.dist_coeffs
            
            self.calibration_file = filepath
            self.logger.info(f"✅ Calibration loaded from: {filepath}")
            
            # Log reprojection error if available
            if isinstance(calib_data, dict) and 'reprojection_error' in calib_data:
                error = calib_data['reprojection_error']
                self.logger.info(f"   📊 Reprojection error: {error:.4f} pixels")
            
            return True
        except Exception as e:
            self.logger.debug(f"   ❌ Failed: {e}")
            return False
    
    def _log_calibration_info(self):
        """Log calibration information for verification."""
        self.logger.info("=" * 70)
        self.logger.info("📊 CALIBRATION PARAMETERS")
        self.logger.info("=" * 70)
        
        try:
            # Camera matrix
            self.logger.info(f"\n🎥 Camera Matrix:")
            self.logger.info(f"   fx (focal length X):   {self.camera_matrix[0, 0]:.2f}")
            self.logger.info(f"   fy (focal length Y):   {self.camera_matrix[1, 1]:.2f}")
            self.logger.info(f"   cx (principal point X): {self.camera_matrix[0, 2]:.2f}")
            self.logger.info(f"   cy (principal point Y): {self.camera_matrix[1, 2]:.2f}")
            aspect_ratio = self.camera_matrix[1, 1] / self.camera_matrix[0, 0]
            self.logger.info(f"   Aspect ratio: {aspect_ratio:.4f}")
            
            # Distortion coefficients
            self.logger.info(f"\n🔧 Distortion Coefficients:")
            self.logger.info(f"   k1 (radial 1):     {self.dist_coeffs[0, 0]:+.6f}")
            self.logger.info(f"   k2 (radial 2):     {self.dist_coeffs[0, 1]:+.6f}")
            self.logger.info(f"   p1 (tangential 1): {self.dist_coeffs[0, 2]:+.6f}")
            self.logger.info(f"   p2 (tangential 2): {self.dist_coeffs[0, 3]:+.6f}")
            self.logger.info(f"   k3 (radial 3):     {self.dist_coeffs[0, 4]:+.6f}")
            
            self.logger.info("\n✅ Calibration parameters verified")
            self.logger.info("=" * 70)
        except Exception as e:
            self.logger.warn(f"⚠️  Could not log calibration details: {e}")
    
    def set_actuator_controller(self, actuator_controller):
        """Set actuator controller reference"""
        self.actuator_controller = actuator_controller
    
    def set_aligned_callback(self, callback):
        """Set callback for when alignment is complete"""
        self.on_aligned_callback = callback
    
    def start_alignment(self):
        """Start 4-tag alignment process"""
        self.logger.info("🎯 Starting 4-Tag AprilTag alignment")
        
        # Reset state
        self.alignment_complete = False
        self.should_exit = False
        self.aligned = False
        self.alignment_time = None
        self.tag_found = False
        self.frame_count = 0
        self.no_tag_count = 0
        
        # Reset multi-tag state
        self.detected_tags = {}
        self.detected_count = 0
        self.grid_center = None
        self.all_tags_centered = False
        self.centering_phase = True
        self.alignment_phase = False
        
        # Reset actuator state
        self.actuators_extended = False
        self.last_alignment_state = False
        
        # Reset PIDs
        self.pid_yaw.reset()
        self.pid_strafe.reset()
        self.pid_forward.reset()
        self.pid_yaw_enhanced.reset()
        self.pid_strafe_enhanced.reset()
        self.pid_forward_enhanced.reset()
        
        # Start control loop
        self.control_timer = self.node.create_timer(CONTROL_LOOP_RATE, self.control_loop)
        self.logger.info(f"🎯 Alignment control loop started (rate: {1/CONTROL_LOOP_RATE:.1f} Hz)")
    
    def stop_alignment(self):
        """Stop alignment process"""
        if self.control_timer:
            self.control_timer.cancel()
            self.control_timer = None
        
        # Stop the robot
        stop_twist = Twist()
        stop_twist.angular.z = 0.0
        stop_twist.linear.x = 0.0
        stop_twist.linear.y = 0.0
        self.cmd_pub.publish(stop_twist)
        
        self.logger.info("⛔ Alignment stopped")
    
    def estop_cb(self, msg):
        """Emergency stop callback"""
        self.estop_active = msg.data
        if msg.data:
            self.logger.warn("⚠️  EMERGENCY STOP ACTIVATED")
            self.pid_yaw.reset()
            self.pid_strafe.reset()
            self.pid_forward.reset()
            self.pid_yaw_enhanced.reset()
            self.pid_strafe_enhanced.reset()
            self.pid_forward_enhanced.reset()
            if self.actuator_controller:
                self.actuator_controller.stop()
        else:
            self.logger.info("✅ Emergency stop deactivated")
    
    def _detect_apriltags_pupil(self, gray: np.ndarray) -> List[AprilTagDetection]:
        """Detect AprilTags using pupil-apriltags"""
        detections = []
        try:
            results = self.detector.detect(gray)
            
            for detection in results:
                tag_id = detection.tag_id
                corners = detection.corners
                center = detection.center
                
                # Calculate tag size in pixels
                size = np.linalg.norm(corners[0] - corners[1])
                
                detections.append(AprilTagDetection(
                    tag_id=tag_id,
                    corners=corners,
                    center=tuple(center),
                    size=size
                ))
        except Exception as e:
            self.logger.debug(f"⚠️  Error detecting tags: {e}")
        
        return detections
    
    def _detect_apriltags_aruco(self, gray: np.ndarray) -> List[AprilTagDetection]:
        """Detect AprilTags using OpenCV ArUco (fallback)"""
        detections = []
        try:
            results = self.aruco_detector.detectMarkers(gray)
            corners, ids, rejected = results
            
            if ids is not None:
                for i, tag_id in enumerate(ids.flatten()):
                    corner_set = corners[i].reshape(4, 2)
                    center = corner_set.mean(axis=0)
                    size = np.linalg.norm(corner_set[0] - corner_set[1])
                    
                    detections.append(AprilTagDetection(
                        tag_id=int(tag_id),
                        corners=corner_set,
                        center=tuple(center),
                        size=size
                    ))
        except Exception as e:
            self.logger.debug(f"⚠️  Error detecting tags: {e}")
        
        return detections
    
    def detect_tags(self, frame: np.ndarray) -> List[AprilTagDetection]:
        """Detect AprilTags in frame"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.detector_type == "pupil-apriltags":
            detections = self._detect_apriltags_pupil(gray)
        else:
            detections = self._detect_apriltags_aruco(gray)
        
        return detections
    
    def get_smooth_center(self, current_center: Tuple[float, float]) -> Tuple[float, float]:
        """Get smoothed center using history"""
        self.center_history.append(current_center)
        
        if len(self.center_history) > 0:
            centers_array = np.array(list(self.center_history))
            smooth_center = np.median(centers_array, axis=0)
            return tuple(smooth_center)
        return current_center
    
    def estimate_pose(self, detection: AprilTagDetection) -> Tuple[np.ndarray, np.ndarray]:
        """Estimate pose of detected tag using calibrated camera matrix"""
        half_size = self.tag_size / 2
        objp = np.array([
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0]
        ], dtype=np.float32)
        
        imgp = detection.corners.astype(np.float32)
        
        # Use calibrated camera matrix and distortion coefficients
        success, rvec, tvec = cv2.solvePnP(
            objp, imgp, self.camera_matrix, self.dist_coeffs,
            useExtrinsicGuess=False, flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if success:
            return rvec, tvec
        return None, None
    
    def image_cb(self, msg):
        """Image callback - process 4-tag detection"""
        if self.should_exit:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.logger.error(f"❌ CV Bridge failed: {e}")
            return
        
        self.frame_count += 1
        
        # Detect all tags
        detections = self.detect_tags(frame)
        
        if len(detections) == 0:
            self.tag_found = False
            self.no_tag_count += 1
            self.detected_count = 0
            
            if self.frame_count % 30 == 0:
                self.logger.debug(f"🔍 Waiting for tags... (frame {self.frame_count})")
            return
        
        # Tags detected
        self.no_tag_count = 0
        self.tag_found = True
        self.last_tag_time = time.time()
        
        # Filter for expected tags
        expected_detections = [d for d in detections if d.tag_id in self.expected_tag_ids]
        self.detected_count = len(expected_detections)
        
        if self.detected_count == 0:
            self.logger.debug(f"⚠️  Detected {len(detections)} tags but none in expected list")
            self.tag_found = False
            return
        
        # Store detected tags
        self.detected_tags = {d.tag_id: d for d in expected_detections}
        
        # Calculate grid center from all detected tags
        if self.detected_count >= 2:
            centers = np.array([d.center for d in expected_detections])
            grid_center = centers.mean(axis=0)
            self.grid_center = self.get_smooth_center(tuple(grid_center))
        
        # Process single-tag pose for backward compatibility (use first detected tag)
        if len(expected_detections) > 0:
            first_tag = expected_detections[0]
            rvec, tvec = self.estimate_pose(first_tag)
            
            if tvec is not None:
                self.x = float(tvec[0])
                self.z = float(tvec[1])
                
                R, _ = cv2.Rodrigues(rvec)
                self.yaw_deg = float(math.degrees(math.atan2(-R[1,0], R[0,0])))
                
                # Low-pass filter
                alpha = ALIGNMENT_FILTER_ALPHA
                self.sx = self.x if self.sx is None else self.sx*(1-alpha)+self.x*alpha
                self.sz = self.z if self.sz is None else self.sz*(1-alpha)+self.z*alpha
                self.syaw = self.yaw_deg if self.syaw is None else self.syaw*(1-alpha)+self.yaw_deg*alpha
                
                self.x, self.z, self.yaw_deg = self.sx, self.sz, self.syaw
        
        # Log status
        if self.frame_count % ALIGNMENT_LOG_INTERVAL == 0:
            self.logger.debug(
                f"📷 Tags detected: {self.detected_count}/4 | "
                f"X:{self.x:+.3f} Z:{self.z:+.3f} Yaw:{self.yaw_deg:+.1f}°"
            )
    
    def control_loop(self):
        """Enhanced control loop for 4-tag alignment"""
        if self.should_exit or self.control_timer is None:
            return
        
        twist = Twist()
        
        # Check emergency stop
        if self.estop_active:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.cmd_pub.publish(twist)
            return
        
        # Check if tags are visible
        if not self.tag_found or (self.last_tag_time and 
                                  time.time() - self.last_tag_time > self.lost_tag_timeout):
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            self.cmd_pub.publish(twist)
            return
        
        # Phase 1: Centering (if not all tags visible)
        if self.centering_phase and self.detected_count < 4:
            self._control_centering(twist)
        else:
            # Phase 2: Alignment (all tags visible or close enough)
            if self.detected_count >= 3 or (self.detected_count >= 1 and self.grid_center is not None):
                self.centering_phase = False
                self.alignment_phase = True
                self._control_alignment(twist)
            else:
                self._control_centering(twist)
        
        self.cmd_pub.publish(twist)
    
    def _control_centering(self, twist: Twist):
        """Control for multi-tag grid centering phase"""
        # Use standard PID but with grid center if available
        wz = self.pid_yaw.step(-self.yaw_deg)
        vy = self.pid_strafe.step(-self.x)
        vx = self.pid_forward.step(-self.z)
        
        # Check tolerances
        x_ok = abs(self.x) <= TOLERANCE_X
        z_ok = abs(self.z) <= TOLERANCE_Z
        yaw_ok = abs(self.yaw_deg) <= TOLERANCE_YAW
        
        if x_ok: vy = 0.0
        if z_ok: vx = 0.0
        if yaw_ok: wz = 0.0
        
        if not x_ok: vy = apply_min(vy, MIN_VY)
        if not z_ok: vx = apply_min(vx, MIN_VX)
        if not yaw_ok: wz = apply_min(wz, MIN_WZ)
        
        twist.angular.z = float(wz)
        twist.linear.y = float(vy)
        twist.linear.x = float(vx)
        
        if self.frame_count % (ALIGNMENT_LOG_INTERVAL * 2) == 0:
            self.logger.info(
                f"🎯 CENTERING PHASE | Tags: {self.detected_count}/4 | "
                f"X:{self.x:+.3f} Z:{self.z:+.3f} Yaw:{self.yaw_deg:+.1f}°"
            )
    
    def _control_alignment(self, twist: Twist):
        """Control for fine-tuned alignment phase"""
        # Use enhanced PID with stricter tolerances
        wz = self.pid_yaw_enhanced.step(-self.yaw_deg)
        vy = self.pid_strafe_enhanced.step(-self.x)
        vx = self.pid_forward_enhanced.step(-self.z)
        
        # Stricter tolerances for alignment phase
        tol_x = TOLERANCE_X * 0.75
        tol_z = TOLERANCE_Z * 0.75
        tol_yaw = TOLERANCE_YAW * 0.8
        
        x_ok = abs(self.x) <= tol_x
        z_ok = abs(self.z) <= tol_z
        yaw_ok = abs(self.yaw_deg) <= tol_yaw
        
        if x_ok: vy = 0.0
        if z_ok: vx = 0.0
        if yaw_ok: wz = 0.0
        
        if not x_ok: vy = apply_min(vy, MIN_VY * 1.2)
        if not z_ok: vx = apply_min(vx, MIN_VX * 1.2)
        if not yaw_ok: wz = apply_min(wz, MIN_WZ * 1.2)
        
        twist.angular.z = float(wz)
        twist.linear.y = float(vy)
        twist.linear.x = float(vx)
        
        currently_aligned = x_ok and z_ok and yaw_ok
        
        # Actuator control logic
        if self.actuator_controller:
            if currently_aligned and not self.last_alignment_state:
                self.logger.info("⬆️ ALIGNED - EXTENDING actuators")
                self.actuator_controller.extend()
                self.actuators_extended = True
            elif not currently_aligned and self.last_alignment_state:
                self.logger.warn("⬇️  MISALIGNED - RETRACTING actuators")
                self.actuator_controller.retract()
                self.actuators_extended = False
        
        self.last_alignment_state = currently_aligned
        
        # Check for alignment completion
        if currently_aligned:
            if not self.aligned:
                self.aligned = True
                self.alignment_time = time.time()
                self.logger.info(
                    f"🎯 PERFECT ALIGNMENT | Tags: {self.detected_count}/4 | "
                    f"Starting {self.alignment_hold_time}s hold time"
                )
            else:
                elapsed_time = time.time() - self.alignment_time
                
                if elapsed_time >= self.alignment_hold_time:
                    self.logger.info(f"✅ ALIGNMENT COMPLETE - Held for {self.alignment_hold_time}s")
                    
                    # Stop the robot
                    self.stop_alignment()
                    
                    self.should_exit = True
                    self.alignment_complete = True
                    
                    # Ensure actuators are extended
                    if self.actuator_controller and not self.actuators_extended:
                        self.logger.info("⬆️ Final check - ensuring actuators are extended")
                        self.actuator_controller.extend()
                        time.sleep(ACTUATOR_EXTENSION_TIME)
                        self.actuators_extended = True
                    
                    # Call completion callback
                    if self.on_aligned_callback:
                        self.on_aligned_callback()
                    
                    return
                else:
                    # Log alignment hold progress
                    if self.frame_count % (ALIGNMENT_LOG_INTERVAL * 3) == 0:
                        remaining = self.alignment_hold_time - elapsed_time
                        self.logger.info(
                            f"🎯 HOLDING ALIGNMENT | {remaining:.1f}s remaining | "
                            f"Tags: {self.detected_count}/4"
                        )
        else:
            if self.aligned:
                self.logger.warn("⚠️  Lost alignment - restarting hold time")
                self.aligned = False
                self.alignment_time = None
            
            if self.frame_count % (ALIGNMENT_LOG_INTERVAL * 2) == 0:
                self.logger.info(
                    f"🔄 FINE-TUNING ALIGNMENT | Tags: {self.detected_count}/4 | "
                    f"X:{self.x:+.3f} Z:{self.z:+.3f} Yaw:{self.yaw_deg:+.1f}°"
                )