import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('s2_wifi_bridge')
    default_map_file = os.path.join(pkg_share, 'maps', 'demo_map.yaml')
    # Fallback to existing map if demo_map does not exist yet
    if not os.path.exists(default_map_file):
        default_map_file = os.path.join(pkg_share, 'maps', 'plan_c_final_map.yaml')

    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params_kinematic.yaml')

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file'
    )

    auto_arg = DeclareLaunchArgument(
        'autonomous_mode',
        default_value='false',
        description='Enable physical autonomous motor driving (true) or safe dry-run (false)'
    )

    autonomous_mode = LaunchConfiguration('autonomous_mode')

    # 1. EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    # 2. Static TF: base_link -> laser_frame
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']
    )

    # 3. Static TF: base_link -> imu_link
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.08',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
    )

    # 4. Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False
        }]
    )

    # 5. AMCL Node (Nguồn DUY NHẤT phát dynamic TF map -> odom)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')]
    )

    # 6. Lifecycle Manager for Localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # 7. Stationary AMCL Auto-Update Node
    stationary_amcl_node = Node(
        package='s2_wifi_bridge',
        executable='stationary_amcl_update_node',
        name='stationary_amcl_update_node',
        output='screen',
        parameters=[{
            'stationary_duration': 3.0,
            'linear_threshold': 0.015,
            'angular_threshold': 0.030,
            'rearm_linear_threshold': 0.035,
            'rearm_angular_threshold': 0.080
        }]
    )

    # 8A. Nav2 Controller Server — AUTONOMOUS MODE
    controller_server_auto = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        condition=IfCondition(autonomous_mode)
    )

    # 8B. Nav2 Controller Server — DRY RUN MODE
    controller_server_dry = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', '/kinematic_controller/cmd_vel_candidate')],
        condition=UnlessCondition(autonomous_mode)
    )

    # 9. Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # 10. Nav2 Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', '/kinematic_controller/behavior_cmd_vel_candidate')]
    )

    # 11. Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )

    # 12. Nav2 Lifecycle Manager for Navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    return LaunchDescription([
        map_file_arg,
        auto_arg,
        ekf_node,
        static_tf_laser,
        static_tf_imu,
        map_server,
        amcl_node,
        lifecycle_manager_localization,
        stationary_amcl_node,
        controller_server_auto,
        controller_server_dry,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation
    ])
