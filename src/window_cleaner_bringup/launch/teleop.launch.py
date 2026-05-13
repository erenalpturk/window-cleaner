from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    NOTE: teleop_twist_keyboard reads from stdin, so this launch file is
    only convenient when launched as a foreground command in its own
    terminal. For most workflows it is simpler to run:

        ros2 run teleop_twist_keyboard teleop_twist_keyboard
    """
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='stdbuf -o L',
        ),
    ])
