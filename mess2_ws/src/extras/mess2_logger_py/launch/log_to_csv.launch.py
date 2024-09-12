
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'topics',
            default_value='[/topic1, /topic2]',
            description='list of topics to log to CSV files'
        ),
        Node(
            package='mess2_logger_py',
            executable='log_to_csv',
            name='log_to_csv',
            output='screen',
            parameters=[{
                'topics': LaunchConfiguration('topics')
            }]
        )
    ])