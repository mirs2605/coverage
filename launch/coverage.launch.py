"""Nav2本体起動済み環境でcoverage_server、bridge、確認用RVizを起動する。

``enable_coverage_server:=false`` はaction serverを起動せずbridge単体で試す場合に使う。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """選択可能なcoverage server、bridge、RVizのlaunch記述を返す。"""
    mirs_share = get_package_share_directory('mirs')
    nav2_params = os.path.join(
        mirs_share, 'config', 'navigation', 'nav2_params.yaml')
    rviz_config = os.path.join(
        get_package_share_directory('coverage'),
        'rviz',
        'coverage.rviz',
    )
    return LaunchDescription([
        DeclareLaunchArgument('zone_topic', default_value='/cleaning_zone'),
        DeclareLaunchArgument('state_topic', default_value='/coverage/state'),
        DeclareLaunchArgument('behavior_tree', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'enable_coverage_server', default_value='true',
            description='Start opennav_coverage server (falseでbridge単体起動)',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Whether to start RViz with coverage.rviz',
        ),
        LogInfo(msg='[coverage] starting coverage_server + bridge...'),
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
