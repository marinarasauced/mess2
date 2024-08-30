
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_rviz2",
            default_value=PathJoinSubstitution([
                FindPackageShare("mess2_visualizer_cpp"),
                "config.rviz"
            ]),
            description="config.rviz"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace="mess2",
            output="screen",
            arguments=["-d", LaunchConfiguration("config_rviz2")],
            parameters=[{"config_file": LaunchConfiguration("config_rviz2")}],
        ),
        Node(
            package="mess2_visualizer_cpp",
            executable="threat_visualizer",
            namespace="mess2",
            output="screen",
        ),
        Node(
            package="mess2_visualizer_py",
            executable="threat_annotator",
            namespace="mess2",
            output="screen",
        ),
    ])
