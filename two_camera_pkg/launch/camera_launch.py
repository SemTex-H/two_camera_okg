from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='usb_cam',
      executable='usb_cam_node_exe',
      namespace='front_camera',
      parameters=[{
        'video_device': '/dev/video0',
        'pixel_format': 'mjpeg2rgb',
        'image_width': 320,
        'image_height': 240,
        'framerate': 15.0
      }]
    ),
    Node(
      package='usb_cam',
      executable='usb_cam_node_exe',
      namespace='rear_camera',
      parameters=[{
        'video_device': '/dev/video2',
        'pixel_format': 'mjpeg2rgb',
        'image_width': 320,
        'image_height': 240,
        'framerate': 15.0
      }]
    )
  ])
