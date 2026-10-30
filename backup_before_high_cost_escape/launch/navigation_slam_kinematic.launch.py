import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('s2_wifi_bridge')
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    # MODE B: Custom Kinematic Controller (Paper Eq. 14)
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params_kinematic.yaml')

    # 1. EKF Node (robot_localization) - Nguồn DUY NHẤT phát dynamic TF odom -> base_link
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

    # 4. SLAM Toolbox Node (Online Asynchronous) - Nguồn DUY NHẤT phát live /map & dynamic TF map -> odom
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {'use_sim_time': False, 'autostart': True}
        ]
    )

    # 5. Lifecycle Manager for SLAM Toolbox
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'bond_timeout': 0.0},
            {'node_names': ['slam_toolbox']}
        ]
    )

    # 6. Nav2 Controller Server (Running plan_c_kinematic_controller in DRY-RUN mode)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', '/kinematic_controller/cmd_vel_candidate')]
    )

    # 7. Nav2 Planner Server (Plans on live evolving SLAM /map)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # 8. Nav2 Behavior Server (Running in DRY-RUN mode)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', '/kinematic_controller/behavior_cmd_vel_candidate')]
    )

    # 9. Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )

    # 10. Nav2 Lifecycle Manager for Navigation Servers
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]}
        ]
    )

    return LaunchDescription([
        ekf_node,
        static_tf_laser,
        static_tf_imu,
        slam_toolbox,
        lifecycle_manager_slam,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation
    ])
