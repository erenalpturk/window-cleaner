from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('window_cleaner_perception')
    params = os.path.join(pkg_dir, 'config', 'perception_params.yaml')

    return LaunchDescription([
        Node(
            package='window_cleaner_perception',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),
        Node(
            package='window_cleaner_perception',
            executable='frame_detector',
            name='frame_detector',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='window_cleaner_perception',
            executable='dirt_segmenter',
            name='dirt_segmenter',
            output='screen',
            parameters=[params],
        ),
    ])
