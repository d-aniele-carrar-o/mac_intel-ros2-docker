from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                'spatial_filter.enable': True,
                'color_width': 1280,
                'color_height': 720,
                'depth_width': 1280,
                'depth_height': 720,
                'color_fps': 6.0,
                'depth_fps': 6.0,
                'publish_tf': False,
            }]
        ),
        Node(
            package='camera_viewer',
            executable='camera_viewer_node',
            name='camera_viewer'
        )
    ])