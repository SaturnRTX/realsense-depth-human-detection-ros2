from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('intel_depth_detection'),
        'rviz',
        'realsense_yolo.rviz'
    )

    return LaunchDescription([
        Node(
            package='intel_depth_detection',
            executable='realsense_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])
