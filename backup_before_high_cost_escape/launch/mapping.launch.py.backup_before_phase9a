import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Thư mục share của package
    pkg_share = get_package_share_directory('s2_wifi_bridge')
    
    # 1. Khai báo các đối số Launch (Position arguments cho LiDAR)
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='LiDAR translation X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='LiDAR translation Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.15', description='LiDAR translation Z')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0', description='LiDAR rotation Roll')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0', description='LiDAR rotation Pitch')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='LiDAR rotation Yaw')

    # Lấy giá trị cấu hình từ các LaunchConfiguration
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Đường dẫn file cấu hình SLAM Toolbox & EKF
    slam_config_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # 2. Khai báo các Node chạy

    # Node 1: EKF Filter Node (robot_localization) - Nguồn DUY NHẤT phát Dynamic TF odom -> base_link
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file]
    )

    # Node 2: Static TF Publisher cho base_link -> laser_frame (Định vị vị trí LiDAR)
    base_link_to_laser_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        arguments=['--x', x, '--y', y, '--z', z, '--roll', roll, '--pitch', pitch, '--yaw', yaw, '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']
    )

    # Node 2b: Static TF Publisher cho base_link -> imu_link (Định vị cảm biến MPU6050)
    base_link_to_imu_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.08', '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
    )

    # Node 3: SLAM Toolbox node (Online Asynchronous)
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

    # Node 4: Lifecycle Manager để tự động configure & activate slam_toolbox
    lifecycle_manager = Node(
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

    # Trả về LaunchDescription chứa toàn bộ cấu hình
    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        ekf_node,
        base_link_to_laser_frame_tf,
        base_link_to_imu_link_tf,
        slam_toolbox,
        lifecycle_manager
    ])
