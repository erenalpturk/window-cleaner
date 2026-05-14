from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('window_cleaner_planning')
    params = os.path.join(pkg_dir, 'config', 'planning_params.yaml')

    return LaunchDescription([
        # The `map -> odom` static TF lives in nav2.launch.py from Sub-phase B
        # onwards. If you're running this launch file standalone (no Nav2),
        # add a static_transform_publisher here yourself or set Foxglove's
        # Fixed frame to `odom` instead of `map`.
        Node(
            package='window_cleaner_planning',
            executable='coverage_planner',
            name='coverage_planner',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='window_cleaner_planning',
            executable='path_follower',
            name='path_follower',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
