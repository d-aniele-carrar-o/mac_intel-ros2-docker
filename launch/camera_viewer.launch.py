from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['/workspace/install/camera_viewer/bin/camera_viewer_node'],
            output='screen'
        )
    ])