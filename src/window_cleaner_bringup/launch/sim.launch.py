from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_desc = FindPackageShare('window_cleaner_description')
    pkg_worlds = FindPackageShare('window_cleaner_worlds')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    xacro_path = PathJoinSubstitution([pkg_desc, 'urdf', 'robot.urdf.xacro'])
    world_path = PathJoinSubstitution([pkg_worlds, 'worlds', 'glass_basic.sdf'])

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Path to the .sdf world file',
    )

    # Apple Silicon XQuartz cannot render OGRE2 GUI / RViz GLX, so default to
    # headless Gazebo + Foxglove WebSocket bridge (open Foxglove Studio on the
    # Mac and connect to ws://localhost:8765).
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Show Gazebo client window (Linux host only)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Auto-start RViz2 (Linux host only)',
    )
    foxglove_arg = DeclareLaunchArgument(
        'foxglove',
        default_value='true',
        description='Start foxglove_bridge on ws://0.0.0.0:8765',
    )

    spawn_x = DeclareLaunchArgument('x', default_value='-2.0')
    spawn_y = DeclareLaunchArgument('y', default_value='0.0')
    spawn_z = DeclareLaunchArgument('z', default_value='0.05')

    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[PathJoinSubstitution([pkg_worlds, 'worlds']), ':',
               PathJoinSubstitution([pkg_worlds])]
    )

    robot_description = {'robot_description': Command(['xacro ', xacro_path])}

    gz_args = PythonExpression([
        "'", LaunchConfiguration('world'), " -r -v 3",
        "' + ('' if '", LaunchConfiguration('gui'), "'.lower() in ('true','1','yes') else ' -s --headless-rendering')",
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('foxglove')),
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'use_sim_time': True,
            'send_buffer_limit': 10_000_000,
        }],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'window_cleaner',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/robot/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/robot/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/robot/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
    )

    # Gazebo sensor plugins publish topics with their own scoped frame_ids
    # (e.g. window_cleaner/base_footprint/lidar) that do not exist in our URDF
    # TF tree. Bridge those Gazebo sensor frames to the URDF link frames so
    # RViz/Foxglove can render the sensor data.
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        arguments=['0', '0', '0', '0', '0', '0',
                   'lidar_link', 'window_cleaner/base_footprint/lidar'],
    )
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_frame_bridge',
        arguments=['0', '0', '0', '0', '0', '0',
                   'camera_optical', 'window_cleaner/base_footprint/rgb_camera'],
    )

    return LaunchDescription([
        world_arg,
        gui_arg, rviz_arg, foxglove_arg,
        spawn_x, spawn_y, spawn_z,
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
        lidar_tf,
        camera_tf,
        rviz,
        foxglove,
    ])
