from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Parameters
    camera_matrix_arg = DeclareLaunchArgument(
        'camera_matrix_path',
        default_value='/path/to/camera_matrix.npy',
        description='Path to camera matrix file'
    )
    
    dist_coeffs_arg = DeclareLaunchArgument(
        'dist_coeffs_path',
        default_value='/path/to/dist_coeffs.npy',
        description='Path to distortion coefficients file'
    )

    # USB Camera
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[
            {"video_device": "/dev/video0"},
            {"image_width": 640},
            {"image_height": 480},
            {"pixel_format": "mjpeg"},
            {"io_method": "mmap"},
            {"camera_frame_id": "camera_link"},
            {"framerate": 30.0}
        ],
        output="screen"
    )

    # AprilTag Align Node
    apriltag_align_node = Node(
        package='mp400_apriltag_align',
        executable='apriltag_align',
        name='apriltag_align',
        parameters=[
            {"image_topic": "/usb_cam/image_raw"},
            {"cmd_vel_topic": "/cmd_vel"},
            {"estop_topic": "/emergency_stop_state"},
            {"alignment_status_topic": "/apriltag/alignment_status"},
            {"show_debug_image": True},
            {"camera_matrix_path": LaunchConfiguration('camera_matrix_path')},
            {"dist_coeffs_path": LaunchConfiguration('dist_coeffs_path')}
        ],
        output="screen"
    )

    return LaunchDescription([
        camera_matrix_arg,
        dist_coeffs_arg,
        usb_cam_node,
        apriltag_align_node
    ])