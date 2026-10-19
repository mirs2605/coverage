"""Nav2本体起動済み環境でcoverage_server、bridge、確認用RVizを起動する。

``enable_coverage_server:=false`` はaction serverを起動せずbridge単体で試す場合に使う。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """選択可能なcoverage server、bridge、RVizのlaunch記述を返す。"""
    mirs_share = FindPackageShare('mirs')
    coverage_share = FindPackageShare('coverage')
    default_map = PathJoinSubstitution([mirs_share, 'maps', 'youkanato.yaml'])
    nav2_params = PathJoinSubstitution(
        [mirs_share, 'config', 'navigation', 'nav2_params.yaml'])
    ekf_config_file = PathJoinSubstitution(
        [mirs_share, 'config', 'ekf', 'ekf_params.yaml'])
    default_behavior_tree = PathJoinSubstitution(
        [coverage_share, 'behavior_trees', 'navigate_coverage_w_recovery.xml'])
    rviz_config = PathJoinSubstitution([coverage_share, 'rviz', 'coverage.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('zone_topic', default_value='/cleaning_zone'),
        DeclareLaunchArgument('state_topic', default_value='/coverage/state'),
        DeclareLaunchArgument('behavior_tree', default_value=default_behavior_tree),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params),
        DeclareLaunchArgument(
            'enable_mirs', default_value='false',
            description='Start mirs hardware and base nodes',
        ),
        DeclareLaunchArgument('esp_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='256000'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_micro_ros', default_value='true'),
        DeclareLaunchArgument('enable_ekf_local', default_value='true'),
        DeclareLaunchArgument('ekf_config_file', default_value=ekf_config_file),
        DeclareLaunchArgument('enable_static_base_link_to_laser_tf', default_value='true'),
        DeclareLaunchArgument(
            'enable_static_base_footprint_to_base_link_tf', default_value='true'),
        DeclareLaunchArgument('enable_static_base_link_to_imu_link_tf', default_value='false'),
        DeclareLaunchArgument(
            'enable_coverage_server', default_value='true',
            description='Start opennav_coverage server (falseでbridge単体起動)',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Whether to start RViz with coverage.rviz',
        ),
        LogInfo(msg='[coverage] starting coverage_server + bridge...'),
        # mirs_launchは機体・センサー・TFを起動し、Nav2/SLAM/RVizは起動しない。
        IncludeLaunchDescription(
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
                'map': LaunchConfiguration('map'),
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
                'enable_rviz': 'false',
                'enable_nav2': 'true',
                'enable_slam_toolbox': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_mirs')),
        ),
        # enable_coverage_server=trueのときcoverage task serverを起動する。
        Node(
            package='opennav_coverage',
            executable='opennav_coverage',
            name='coverage_server',
            output='screen',
            parameters=[
                nav2_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            condition=IfCondition(LaunchConfiguration('enable_coverage_server')),
        ),
        # task serverを起動するときだけlifecycle管理対象へ登録する。
        Node(
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
        ),
        Node(
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
        ),
        # use_rviz=trueのときcoverage確認用設定でRVizを起動する。
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),
    ])
