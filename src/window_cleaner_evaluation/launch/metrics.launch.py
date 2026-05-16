"""Metrics node bring-up (Phase 4).

Convenience/debug launcher for a single mission's metrics collection. The
benchmark sweep (`scripts/run_benchmark.sh`) starts `metrics_node` directly
via `ros2 run ... --ros-args` so it can inject per-world bounds and the
run index; this launch file mirrors that for manual single runs.

    ros2 launch window_cleaner_evaluation metrics.launch.py \
        params_file:=<...>/config/metrics_params.yaml run_id:=1

`use_sim_time` is forced True: every duration is measured off `/clock`
because Gazebo runs below 1.0 RTF on Apple Silicon, so wall-clock would be
wrong.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('window_cleaner_evaluation')
    default_params = os.path.join(pkg_dir, 'config', 'metrics_params.yaml')

    params_file = LaunchConfiguration('params_file')
    run_id = LaunchConfiguration('run_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Path to the metrics parameter YAML'),
        DeclareLaunchArgument(
            'run_id', default_value='0',
            description='Run index written to the CSV row'),
        Node(
            package='window_cleaner_evaluation',
            executable='metrics_node',
            name='metrics_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': True, 'run_index': run_id},
            ],
        ),
    ])
