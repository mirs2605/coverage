"""MIRS車体、Nav2、coverage server、BLE受信、bridgeをまとめて起動する。

地図生成・coverage経路計算・走行追従は、それぞれSLAM/Nav2/coverage serverが担う。
個別機能はlaunch引数で有効・無効を選択できる。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _arg(name, default, description=''):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description():
    mirs_share = FindPackageShare('mirs')
    coverage_share = FindPackageShare('coverage')

    default_map = PathJoinSubstitution([mirs_share, 'maps', 'es.yaml'])
    ekf_config_file = PathJoinSubstitution(
        [mirs_share, 'config', 'ekf', 'ekf_params.yaml'])
    nav2_params = PathJoinSubstitution(
        [mirs_share, 'config', 'navigation', 'nav2_params.yaml'])
    default_behavior_tree = PathJoinSubstitution(
        [coverage_share, 'behavior_trees', 'navigate_coverage_w_recovery.xml'])
    rviz_config = PathJoinSubstitution([coverage_share, 'rviz', 'coverage.rviz'])

    args = [
        _arg('map', default_map, 'Map yaml for Nav2'),
        _arg('nav2_params_file', nav2_params, 'Nav2 parameter file'),
        _arg('use_sim_time', 'false'),
        _arg('use_rviz', 'true', 'Start RViz with coverage.rviz'),
        _arg('esp_port', '/dev/ttyUSB1'),
        _arg('lidar_port', '/dev/ttyUSB0'),
        _arg('lidar_baudrate', '256000'),
        _arg('lidar_frame', 'laser'),
        _arg('enable_lidar', 'true'),
        _arg('enable_micro_ros', 'true'),
        _arg('enable_hardware', 'true', 'Start mirs base and sensor nodes'),
        _arg('enable_nav2', 'true', 'Include nav2_bringup'),
        _arg('enable_coverage_server', 'true', 'Start opennav_coverage server'),
        _arg('enable_ble', 'true', 'Start ble_receiver_node (falseで手動テスト用)'),
        _arg('enable_bridge', 'true', 'Start coverage_bridge'),
        _arg('enable_monitor', 'true', 'Publish coverage diagnostics'),
        _arg('enable_ekf_local', 'true', 'EKF odom->base_footprint (off時はstatic TFで代用可)'),
        _arg('ekf_config_file', ekf_config_file, 'EKF parameter file'),
        _arg('enable_static_base_link_to_laser_tf', 'true'),
        _arg('enable_static_base_footprint_to_base_link_tf', 'true'),
        _arg('enable_static_base_link_to_imu_link_tf', 'true'),
        _arg('autostart', 'true', 'Autostart Nav2 lifecycle'),
        # nav2_bringupのPythonExpressionで評価されるためPython bool表記を渡す。
        _arg('slam', 'False', 'Set True for SLAM mode (no map_server)'),
        _arg('zone_topic', '/cleaning_zone'),
        _arg('state_topic', '/coverage/state'),
        _arg('behavior_tree', default_behavior_tree,
             'BT XML override (default=recovery BT, empty=server default)'),
    ]

    # mirs_launch.py API経由で機体、Nav2、安全ゲートを起動する。
    mirs_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([mirs_share, 'launch', 'mirs_launch.py'])
        ),
        launch_arguments={
            'esp_port': LaunchConfiguration('esp_port'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'enable_lidar': LaunchConfiguration('enable_lidar'),
            'enable_micro_ros': LaunchConfiguration('enable_micro_ros'),
            'enable_ekf_local': LaunchConfiguration('enable_ekf_local'),
            'ekf_config_file': LaunchConfiguration('ekf_config_file'),
            'enable_static_base_link_to_laser_tf':
                LaunchConfiguration('enable_static_base_link_to_laser_tf'),
            'enable_static_base_footprint_to_base_link_tf':
                LaunchConfiguration('enable_static_base_footprint_to_base_link_tf'),
            'enable_static_base_link_to_imu_link_tf':
                LaunchConfiguration('enable_static_base_link_to_imu_link_tf'),
            'enable_rviz': 'false',
            'enable_nav2': LaunchConfiguration('enable_nav2'),
            'enable_slam_toolbox': 'false',
            'autostart': LaunchConfiguration('autostart'),
            'slam': LaunchConfiguration('slam'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_hardware')),
    )

    # coverage server設定はnav2_params.yamlを使い、時刻設定だけlaunchから渡す。
    # Nav2本体のlifecycle一覧に含まれないため、専用managerでactive化する。
    # enable_coverage_server=trueのときcoverage task serverを起動する。
    coverage_server = Node(
        package='opennav_coverage',
        executable='opennav_coverage',
        name='coverage_server',
        output='screen',
        parameters=[
            LaunchConfiguration('nav2_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(LaunchConfiguration('enable_coverage_server')),
    )

    # serverを起動するときだけ専用lifecycle managerも起動する。
    coverage_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_coverage',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['coverage_server'],
            'bond_timeout': 10.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_coverage_server')),
    )

    # BLEパッケージが公開する起動APIで、JSON受信ノードを起動する。
    ble_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ble_server'), 'launch',
                'ble_receiver.launch.py',
            ])
        ),
        launch_arguments={
            'enabled': 'true',
            'default_frame_id': 'map',
            'cleaning_zone_topic': LaunchConfiguration('zone_topic'),
            'initial_pose_topic': '/initialpose',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_ble')),
    )

    # enable_bridge=trueのときcleaning_zoneをcoverage actionへ中継する。
    bridge = Node(
        package='coverage',
        executable='coverage_bridge',
        name='coverage_bridge',
        output='screen',
        parameters=[{
            'zone_topic': LaunchConfiguration('zone_topic'),
            'state_topic': LaunchConfiguration('state_topic'),
            'behavior_tree': LaunchConfiguration('behavior_tree'),
            'frame_id': 'map',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
    )

    # enable_monitor=trueのとき実機のtopic・TF診断を発行する。
    monitor = Node(
        package='coverage',
        executable='coverage_monitor',
        name='coverage_monitor',
        output='screen',
        parameters=[{
            'profile': 'hardware',
            'zone_topic': LaunchConfiguration('zone_topic'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_monitor')),
    )

    # use_rviz=trueのときcoverage表示設定を読み込んでRVizを起動する。
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='[production] starting hardware...'),
        mirs_bringup,
        LogInfo(msg='[production] starting Nav2 through mirs_launch API...'),
        LogInfo(msg='[production] starting coverage_server/ble/bridge/rviz...'),
        coverage_server,
        coverage_lifecycle,
        ble_receiver,
        bridge,
        monitor,
        rviz,
    ])
