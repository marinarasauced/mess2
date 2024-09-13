from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import json


def make_log_to_csv_node(name_camera):
    """
    
    """
    topic_camera_info = f'/{name_camera}/camera_info'
    # topic_image_raw = f'/{name_camera}/image_raw'
    topic_meta = f'/{name_camera}/meta'

    topics = [topic_camera_info, topic_meta]
    print(topics)

    node = Node(
        package='mess2_logger_py',
        executable='log_to_csv',
        name=f'log_{name_camera}_to_csv',
        output='screen',
        parameters=[
            {'topics': topics},
            {'name_actor': name_camera},
            {'dir_logs': LaunchConfiguration('dir_logs')},
            {'dir_sub': LaunchConfiguration('dir_sub_flir')}
        ],
    )
    return node


def make_log_to_jpg_node():
    """
    
    """
    node = Node(
        package='mess2_logger_cpp',
        executable='log_to_jpg',
        name='log_flir_to_jpg',
        output='screen',
        parameters=[
            {'path_config': LaunchConfiguration('path_config')},
            {'dir_logs': LaunchConfiguration('dir_logs')},
            {'dir_sub': LaunchConfiguration('dir_sub_flir')}
        ],
    )
    return node


def launch_setup(context, *args, **kwargs):
    """
    Create multiple camera nodes.
    """
    name_cameras_str = LaunchConfiguration('name_cameras').perform(context)
    name_cameras = json.loads(name_cameras_str)
    nodes = [make_log_to_jpg_node()]
    for name_camera in name_cameras:
        nodes.append(make_log_to_csv_node(name_camera))
    return nodes


def generate_launch_description():
    """
    Create composable node by calling opaque function.
    """
    return LaunchDescription(
        [
            LaunchArg(
                'path_config',
                default_value='/home/mess2/mess2/mess2_ws/install/spinnaker_camera_driver/share/spinnaker_camera_driver/config/_cameras.yaml',
                description='path to the YAML file containing camera configurations.',
            ),
            LaunchArg(
                'name_cameras',
                default_value='["flir1", "flir2"]',
                description='list of cameras to log'
            ),
            LaunchArg(
                'dir_logs',
                default_value='/home/mess2/mess2/logs/2024_09_12/testing',
                description='directory path where images will be saved.',
            ),
            LaunchArg(
                'dir_sub_flir',
                default_value='flir',
                description='sub directory for flir data'
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
