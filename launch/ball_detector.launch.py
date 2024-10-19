from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ball_detector'),
        'config',
        'config.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('ball_detector'),
        'rviz',
        'tennis.rviz'
    )
    return LaunchDescription([
        Node(
            package='ball_detector',
            executable='ball_detector',
            name='ball_detector',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])