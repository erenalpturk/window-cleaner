from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('window_cleaner_description')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])

    use_jsp_gui = DeclareLaunchArgument(
        'use_jsp_gui',
        default_value='true',
        description='Use joint_state_publisher_gui to wiggle wheel joints',
    )

    robot_description = {'robot_description': Command(['xacro ', xacro_path])}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_jsp_gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return LaunchDescription([
        use_jsp_gui,
        robot_state_publisher,
        jsp_gui,
        rviz,
    ])
