"""coverage.launch.py: coverage_server + bridge + 確認用RViz の最小起動。

Nav2本体は別途起動済み(またはproduction.launch.pyを使う)想定。
opennav_coverage単体の動作確認用。

起動例:
    ros2 launch coverage coverage.launch.py
    ros2 launch coverage coverage.launch.py use_rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
            'use_rviz', default_value='true',
            description='Whether to start RViz with coverage.rviz',
        ),
        LogInfo(msg='[coverage] starting coverage_server + bridge...'),
        Node(
            package='opennav_coverage',
            executable='opennav_coverage',
            name='coverage_server',
            output='screen',
            parameters=[
                nav2_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
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
