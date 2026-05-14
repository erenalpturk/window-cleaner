from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('window_cleaner_control')
    params = os.path.join(pkg_dir, 'config', 'control_params.yaml')

    return LaunchDescription([
        Node(
            package='window_cleaner_control',
            executable='cleaning_controller',
            name='cleaning_controller',
            output='screen',
            parameters=[params],
        ),
    ])
