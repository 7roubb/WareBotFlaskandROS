from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # USB Camera (usb_cam package)
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

    # Apriltag Align Node
    apriltag_align_node = Node(
        package='mp400_apriltag_align',
        executable='apriltag_align',
        name='apriltag_align',
        parameters=[
            {"image_topic": "/usb_cam/image_raw"},
            {"cmd_vel_topic": "/cmd_vel"},
            {"estop_topic": "/emergency_stop_state"},
            {"show_debug_image": True}
        ],
        output="screen"
    )

    return LaunchDescription([
        usb_cam_node,
        apriltag_align_node
    ])
