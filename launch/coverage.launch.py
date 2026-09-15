"""coverage.launch.py: mirs_coverage executor + 確認用RViz の起動構成。

RViz設定は本パッケージ内の rviz/cleaning_zone.rviz に完結する。
mirs 側へのBLE実装の混入はない。
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('mirs_coverage'),
        'rviz',
        'cleaning_zone.rviz',
    )
    return LaunchDescription([
        DeclareLaunchArgument('zone_topic', default_value='/cleaning_zone'),
        DeclareLaunchArgument('costmap_topic', default_value='/global_costmap/costmap'),
        DeclareLaunchArgument('spacing', default_value='0.35'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Whether to start RViz with cleaning_zone.rviz',
        ),
        Node(
            package='mirs_coverage',
            executable='coverage_executor',
            name='coverage_executor',
            parameters=[{
                'zone_topic': LaunchConfiguration('zone_topic'),
                'costmap_topic': LaunchConfiguration('costmap_topic'),
                'spacing': LaunchConfiguration('spacing'),
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen',
        ),
    ])
