"""Nav2 stack bring-up for the window cleaner.

Launches map_server, controller_server, planner_server, smoother_server,
behavior_server and bt_navigator under a single lifecycle_manager that
auto-starts all of them. Also publishes the identity `map -> odom` static
TF (Nav2 has no AMCL — odom is ground-truth from Ignition DiffDrive and
the robot spawns at world (0, 0, 0.05) which coincides with the map origin).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('window_cleaner_bringup')
    nav2_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(bringup_share, 'maps', 'glass_basic.yaml')
    # Custom NavigateThroughPoses BT: the Nav2 default prunes the next return
    # strip's near corner (RemovePassedGoals radius=0.7 > strip spacing
    # 0.2875 m), making the robot corner-cut every U-turn. Our fork sets
    # radius=0.15. Resolved from the install share dir so the path is correct
    # inside the container regardless of the host workspace location.
    nav_through_poses_bt = os.path.join(
        bringup_share, 'config', 'bt', 'navigate_through_poses_coverage.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_arg = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    odom_offset_x = LaunchConfiguration('odom_offset_x')
    odom_offset_y = LaunchConfiguration('odom_offset_y')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo /clock as the time source')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Auto-start the Nav2 lifecycle manager')
    declare_map_yaml = DeclareLaunchArgument(
        'map', default_value=map_yaml,
        description='Path to the map YAML for map_server')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=nav2_params,
        description='Path to the Nav2 parameter file')
    # Phase 4 (additive): the `map -> odom` offset must equal the spawn
    # position (see the comment on map_to_odom below). Defaults reproduce the
    # exact pre-Phase-4 behaviour for the 5x3 worlds; run_benchmark.sh passes
    # the per-world SW-corner spawn for `glass_small` (whose surface is too
    # small for the hard-coded -2.15/-1.15).
    declare_odom_offset_x = DeclareLaunchArgument(
        'odom_offset_x', default_value='-2.15',
        description='map->odom x offset; must match the sim spawn x')
    declare_odom_offset_y = DeclareLaunchArgument(
        'odom_offset_y', default_value='-1.15',
        description='map->odom y offset; must match the sim spawn y')

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    # The Ignition DiffDrive plugin makes `odom` the spawn pose's frame, so
    # `odom -> base_footprint` is the robot's delta from spawn. To keep Nav2
    # planning in true world coordinates we offset `map -> odom` by the
    # spawn position. Sim.launch.py spawns at (-2.15, -1.15) (the SW corner,
    # so the boustrophedon plan starts right under the robot), and this
    # static TF undoes that offset so map-frame (0,0) is still world (0,0).
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=[odom_offset_x, odom_offset_y, '0', '0', '0', '0',
                   'map', 'odom'],
        output='screen',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_arg,
        }],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        # The dict is applied after params_file, so it overrides the
        # default_nav_through_poses_bt_xml value (unset in nav2_params.yaml,
        # which otherwise falls back to the upstream radius=0.7 tree).
        parameters=[
            params_file,
            {'default_nav_through_poses_bt_xml': nav_through_poses_bt},
        ],
    )

    # Delay the lifecycle manager slightly so all servers have advertised
    # their /change_state services by the time it starts the cascade. This
    # avoids a race we saw on Apple Silicon Docker where the manager would
    # try to configure servers that hadn't finished initialising.
    lifecycle_manager = TimerAction(
        period=2.0,
        actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 0.0,
            }],
        )],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_map_yaml,
        declare_params_file,
        declare_odom_offset_x,
        declare_odom_offset_y,
        map_to_odom,
        map_server,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
