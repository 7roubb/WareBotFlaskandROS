from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    COMBINED Launch File: USB Camera + Integrated Task Runner + Linear Actuators + Footprint Switcher
    
    MODIFIED: ONLY calibration_file_path parameter (no separate matrix/coeffs files)
    """
    
    # Get package share directory
    warebot_pkg_share = get_package_share_directory('warebot_task_runner')
    
    # ====== LAUNCH ARGUMENTS ======
    
    # Camera arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path (e.g., /dev/video0, /dev/video1)'
    )
    
    # Task runner arguments
    robot_id_arg = DeclareLaunchArgument(
        "robot_id",
        default_value="robot_1",
        description="Unique ID of the robot"
    )
    
    mqtt_host_arg = DeclareLaunchArgument(
        "mqtt_host",
        default_value="localhost",
        description="MQTT broker hostname"
    )
    
    mqtt_port_arg = DeclareLaunchArgument(
        "mqtt_port",
        default_value="1884",
        description="MQTT broker port"
    )
    
    mqtt_qos_arg = DeclareLaunchArgument(
        "mqtt_qos",
        default_value="1",
        description="MQTT QoS level"
    )
    
    mqtt_reconnect_base_arg = DeclareLaunchArgument(
        "mqtt_reconnect_base",
        default_value="1.0",
        description="Base reconnection delay in seconds"
    )
    
    mqtt_reconnect_max_arg = DeclareLaunchArgument(
        "mqtt_reconnect_max",
        default_value="30.0",
        description="Maximum reconnection delay in seconds"
    )
    
    enable_hardware_arg = DeclareLaunchArgument(
        "enable_hardware",
        default_value="true",
        description="Enable hardware controllers (set to false for simulation)"
    )
    
    # AprilTag parameters
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/image_raw",
        description="Image topic for AprilTag detection"
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Command velocity topic"
    )
    
    estop_topic_arg = DeclareLaunchArgument(
        "estop_topic",
        default_value="/emergency_stop_state",
        description="Emergency stop topic"
    )
    
    show_debug_arg = DeclareLaunchArgument(
        "show_debug_image",
        default_value="true",
        description="Enable debug image display for AprilTag detection"
    )
    
    alignment_hold_time_arg = DeclareLaunchArgument(
        "alignment_hold_time",
        default_value="30.0",
        description="Time in seconds to hold alignment before proceeding"
    )
    
    # ========================================================================
    # MODIFIED: ONLY calibration_file_path parameter (no separate matrix/coeffs)
    # ========================================================================
    calibration_file_path_arg = DeclareLaunchArgument(
        "calibration_file_path",
        default_value=os.path.join(warebot_pkg_share, "config", "calibration.npy"),
        description="Full path to calibration.npy file"
    )
    
    # LINEAR ACTUATOR PARAMETERS
    actuator_port_arg = DeclareLaunchArgument(
        "actuator_port",
        default_value="/dev/ttyUSB1",
        description="Serial port for linear actuator controller"
    )
    
    actuator_baudrate_arg = DeclareLaunchArgument(
        "actuator_baudrate",
        default_value="115200",
        description="Baudrate for actuator serial communication"
    )
    
    # FOOTPRINT SWITCHING PARAMETER
    footprint_service_arg = DeclareLaunchArgument(
        "footprint_service",
        default_value="/switch_footprint",
        description="Service name for footprint switching"
    )
    
    # ====== NODES ======
    
    # USB CAMERA NODE
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='',
        parameters=[
            {"video_device": LaunchConfiguration('video_device')},
            {"image_width": 640},
            {"image_height": 480},
            {"pixel_format": "yuyv"},
            {"io_method": "mmap"},
            {"camera_frame_id": "camera_link"},
            {"framerate": 30.0},
            {"brightness": -1},
            {"contrast": -1},
            {"saturation": -1},
            {"sharpness": -1},
            {"focus_absolute": -1},
            {"auto_exposure": 1},
            {"exposure_absolute": 100},
        ],
        remappings=[
            ('/usb_cam/image_raw', '/image_raw'),
        ],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_hardware")),
    )
    
    # INTEGRATED TASK RUNNER NODE
    task_runner_node = Node(
        package="warebot_task_runner",
        executable="task_runner",
        name="task_runner",
        parameters=[
            # MQTT parameters
            {"robot_id": LaunchConfiguration("robot_id")},
            {"mqtt_host": LaunchConfiguration("mqtt_host")},
            {"mqtt_port": LaunchConfiguration("mqtt_port")},
            {"mqtt_qos": LaunchConfiguration("mqtt_qos")},
            {"mqtt_reconnect_base": LaunchConfiguration("mqtt_reconnect_base")},
            {"mqtt_reconnect_max": LaunchConfiguration("mqtt_reconnect_max")},
            {"enable_hardware": LaunchConfiguration("enable_hardware")},
            
            # AprilTag parameters
            {"image_topic": LaunchConfiguration("image_topic")},
            {"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")},
            {"estop_topic": LaunchConfiguration("estop_topic")},
            {"show_debug_image": LaunchConfiguration("show_debug_image")},
            {"alignment_hold_time": LaunchConfiguration("alignment_hold_time")},
            {"lost_tag_timeout": 0.7},
            
            # ========================================================================
            # MODIFIED: ONLY calibration_file_path (no separate matrix/coeffs)
            # ========================================================================
            {"calibration_file_path": LaunchConfiguration("calibration_file_path")},
            
            # LINEAR ACTUATOR PARAMETERS
            {"actuator_port": LaunchConfiguration("actuator_port")},
            {"actuator_baudrate": LaunchConfiguration("actuator_baudrate")},
            
            # FOOTPRINT SWITCHING PARAMETER
            {"footprint_service": LaunchConfiguration("footprint_service")},
        ],
        output="screen",
        emulate_tty=True,
    )
    
    # ====== RETURN LAUNCH DESCRIPTION ======
    return LaunchDescription([
        # Arguments
        video_device_arg,
        robot_id_arg,
        mqtt_host_arg,
        mqtt_port_arg,
        mqtt_qos_arg,
        mqtt_reconnect_base_arg,
        mqtt_reconnect_max_arg,
        enable_hardware_arg,
        image_topic_arg,
        cmd_vel_topic_arg,
        estop_topic_arg,
        show_debug_arg,
        alignment_hold_time_arg,
        calibration_file_path_arg,  # NEW: Only calibration_file_path
        actuator_port_arg,
        actuator_baudrate_arg,
        footprint_service_arg,
        
        # Nodes
        usb_cam_node,
        task_runner_node,
    ])