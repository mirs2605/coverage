"""coverage.launch.py: mirs_coverage executor の起動構成。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('zone_topic', default_value='/cleaning_zone'),
        DeclareLaunchArgument('costmap_topic', default_value='/global_costmap/costmap'),
        DeclareLaunchArgument('spacing', default_value='0.35'),
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
    ])
