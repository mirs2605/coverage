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
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    coverage_share = FindPackageShare('coverage')

    default_map = PathJoinSubstitution([mirs_share, 'maps', 'youkanato.yaml'])
    ekf_config_file = PathJoinSubstitution(
        [mirs_share, 'config', 'ekf', 'ekf_params.yaml'])
    nav2_params = PathJoinSubstitution(
        [mirs_share, 'config', 'navigation', 'nav2_params.yaml'])
    default_behavior_tree = PathJoinSubstitution(
        [coverage_share, 'behavior_trees', 'navigate_coverage_w_recovery.xml'])
    rviz_config = PathJoinSubstitution([coverage_share, 'rviz', 'coverage.rviz'])

    args = [
        _arg('map', default_map, 'Map yaml for Nav2'),
        _arg('use_sim_time', 'false'),
        _arg('use_rviz', 'true', 'Start RViz with coverage.rviz'),
        _arg('esp_port', '/dev/ttyUSB1'),
        _arg('lidar_port', '/dev/ttyUSB0'),
        _arg('lidar_baudrate', '256000'),
        _arg('lidar_frame', 'laser'),
        _arg('enable_lidar', 'true'),
        _arg('enable_micro_ros', 'true'),
        _arg('enable_hardware', 'true', 'Include mirs_hardware.launch.py'),
        _arg('enable_nav2', 'true', 'Include nav2_bringup'),
        _arg('enable_coverage_server', 'true', 'Start opennav_coverage server'),
        _arg('enable_ble', 'true', 'Start ble_receiver_node (falseで手動テスト用)'),
        _arg('enable_bridge', 'true', 'Start coverage_bridge'),
        _arg('enable_ekf_local', 'true', 'EKF odom->base_footprint (off時はstatic TFで代用可)'),
        _arg('ekf_config_file', ekf_config_file, 'EKF parameter file'),
        _arg('enable_static_base_link_to_laser_tf', 'true'),
        _arg('enable_static_base_footprint_to_base_link_tf', 'true'),
        _arg('enable_static_base_link_to_imu_link_tf', 'false'),
        _arg('autostart', 'true', 'Autostart Nav2 lifecycle'),
        # nav2_bringupのPythonExpressionで評価されるためPython bool表記を渡す。
        _arg('slam', 'False', 'Set True for SLAM mode (no map_server)'),
        _arg('zone_topic', '/cleaning_zone'),
        _arg('state_topic', '/coverage/state'),
        _arg('behavior_tree', default_behavior_tree,
             'BT XML override (default=recovery BT, empty=server default)'),
    ]

    # hardwareと基礎ノードを起動する。Nav2とRVizはcoverage側で管理する。
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
            'enable_nav2': 'false',
            'enable_slam_toolbox': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_hardware')),
    )

    # enable_nav2=trueのとき地図またはSLAM設定でNav2を起動する。
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [nav2_bringup_share, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params,
            'autostart': LaunchConfiguration('autostart'),
            'slam': LaunchConfiguration('slam'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_nav2')),
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
            nav2_params,
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

    # enable_ble=trueのときBLE受信ノードを起動する。
    # cleaning_zone_topic／initial_pose_topicでそれぞれの配信先を上書きできる。
    # initial_poseはAMCLの/initalposeへ直接届く（coverage_bridgeを経由しない）。
    ble_receiver = Node(
        package='ble_server',
        executable='ble_receiver_node',
        name='ble_receiver_node',
        output='screen',
        parameters=[{
            'default_frame_id': 'map',
            'cleaning_zone_topic': LaunchConfiguration('zone_topic'),
            'initial_pose_topic': '/initialpose',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
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
        LogInfo(msg='[production] starting Nav2...'),
        nav2_bringup,
        LogInfo(msg='[production] starting coverage_server/ble/bridge/rviz...'),
        coverage_server,
        coverage_lifecycle,
        ble_receiver,
        bridge,
        rviz,
    ])
