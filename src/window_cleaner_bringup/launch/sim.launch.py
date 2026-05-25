from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    glass_marker_arg = DeclareLaunchArgument(
        'glass_marker',
        default_value='true',
        description='Publish /perception/glass_surface_marker for Foxglove 3D',
    )

    # Spawn at the SW corner so the boustrophedon plan's first waypoint
    # (-2.15, -1.15 in map frame, after coverage_planner's strip_width/2
    # margin) is right next to the robot. Otherwise Nav2 considers the
    # first 1-2 waypoints "behind" the robot at startup and skips them,
    # losing the bottom strip from the coverage sweep.
    #
    # Odometry still publishes deltas from the spawn point — but our
    # ground-truth /odom from the Gazebo DiffDrive plugin reports
    # WORLD-frame coordinates, so map -> odom can remain identity.
    spawn_x = DeclareLaunchArgument('x', default_value='-2.15')
    spawn_y = DeclareLaunchArgument('y', default_value='-1.15')
    spawn_z = DeclareLaunchArgument('z', default_value='0.05')

    # Simple (no-Nav2) mode: publish a static map -> odom TF so the `map`
    # frame exists for Foxglove (coverage path + glass boundary are in
    # `map`). Offset equals the spawn x/y because the Ignition DiffDrive
    # `/odom` is reported relative to the spawn point — identical to what
    # nav2.launch.py did. Set map_odom_tf:=false when running the advanced
    # Nav2 mode (nav2.launch.py publishes its own map -> odom).
    map_odom_tf_arg = DeclareLaunchArgument(
        'map_odom_tf', default_value='true',
        description='Publish static map->odom TF (simple no-Nav2 mode)')

    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[PathJoinSubstitution([pkg_worlds, 'worlds']), ':',
               PathJoinSubstitution([pkg_worlds])]
    )

    robot_description = {'robot_description': ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)}

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

    glass_marker = Node(
        package='window_cleaner_perception',
        executable='glass_surface_marker',
        name='glass_surface_marker',
        output='screen',
        condition=IfCondition(LaunchConfiguration('glass_marker')),
        parameters=[{'use_sim_time': True}],
    )

    # Static map -> odom for the simple no-Nav2 flow (see map_odom_tf_arg).
    # Translation = spawn x/y so map-frame (world) coords line up with the
    # spawn-relative DiffDrive odom. Disabled for the advanced Nav2 mode.
    map_to_odom_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=[LaunchConfiguration('x'), LaunchConfiguration('y'),
                   '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(LaunchConfiguration('map_odom_tf')),
    )

    return LaunchDescription([
        world_arg,
        gui_arg, rviz_arg, foxglove_arg, glass_marker_arg,
        spawn_x, spawn_y, spawn_z,
        map_odom_tf_arg,
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
        lidar_tf,
        camera_tf,
        map_to_odom_static,
        rviz,
        foxglove,
        glass_marker,
    ])
