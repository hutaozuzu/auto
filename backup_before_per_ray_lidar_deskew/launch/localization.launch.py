import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('s2_wifi_bridge')
    default_map_file = os.path.join(pkg_share, 'maps', 'plan_c_final_map.yaml')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file'
    )

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
        arguments=['--x', '0', '--y', '0', '--z', '0.15',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']
    )

    # 3. Static TF: base_link -> imu_link
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['--x', '0', '--y', '0', '--z', '0.08',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
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

    # 5. AMCL Node
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

    return LaunchDescription([
        map_file_arg,
        ekf_node,
        static_tf_laser,
        static_tf_imu,
        map_server,
        amcl_node,
        lifecycle_manager_localization,
        stationary_amcl_node
    ])
