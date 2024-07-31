from os import environ
import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_gz(context, *args, **kwargs):
    """
    Launch the gz sim.
    """

    env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                environ.get('LD_LIBRARY_PATH', default='')])}

    gz_args = LaunchConfiguration('gz_args').perform(context)
    gz_version = LaunchConfiguration('gz_version').perform(context)
    ign_args = LaunchConfiguration('ign_args').perform(context)
    debugger = LaunchConfiguration('debugger').perform(context)
    on_exit_shutdown = LaunchConfiguration('on_exit_shutdown').perform(context)

    if not len(gz_args) and len(ign_args):
        print("ign_args is deprecated, migrate to gz_args!")
        exec_args = ign_args
    else:
        exec_args = gz_args

    exec = 'ruby $(which gz) sim'

    if debugger != 'false':
        debug_prefix = 'x-terminal-emulator -e gdb -ex run --args'
    else:
        debug_prefix = None

    if on_exit_shutdown:
        on_exit = Shutdown()
    else:
        on_exit = None

    return [ExecuteProcess(
            cmd=[exec, exec_args, '--force-version', gz_version],
            output='screen',
            additional_env=env,
            shell=True,
            prefix=debug_prefix,
            on_exit=on_exit
        )]

def generate_launch_description():
    """
    Generate the launch description for the launch file.
    """

    sim = 'cscp.sdf'
    pkg = 'mess2_gazebo'
    path = os.path.join(get_package_share_directory(pkg), f'worlds/{sim}')
    print(path)

    gz_args = DeclareLaunchArgument(
        'gz_args', 
        default_value=f'{path} -r',
        description='gz sim args'
    )

    gz_version = DeclareLaunchArgument(
        'gz_version',
        default_value='8',
        description='gz sim major version'
    )

    ign_args = DeclareLaunchArgument(
        'ign_args', 
        default_value='',
        description='deprecated: gz sim args'
    )

    ign_version = DeclareLaunchArgument(
        'ign_version',
        default_value='8',
        description='deprecated: gz sim major version'
    )

    debugger = DeclareLaunchArgument(
        'debugger',
        default_value='false',
        description='run in debugger'
    )

    on_exit_shutdown = DeclareLaunchArgument(
        'on_exit_shutdown',
        default_value='false',
        description='shutdown on gz sim exit'
    )

    bridge_topics = [
        {
            'gz_topic': '/model/burger1/odometry',
            'ros_topic': '/ugv/burger1/odom',
            'gz_type': 'gz.msgs.Odometry',
            'ros_type': 'nav_msgs/msg/Odometry'
        },
        {
            'gz_topic': '/model/burger1/cmd_vel',
            'ros_topic': '/ugv/burger1/cmd_vel',
            'gz_type': 'gz.msgs.Twist',
            'ros_type': 'geometry_msgs/msg/Twist'
        },
        {
            'gz_topic': '/model/wafflepi1/odometry',
            'ros_topic': '/ugv/wafflepi1/odom',
            'gz_type': 'gz.msgs.Odometry',
            'ros_type': 'nav_msgs/msg/Odometry'
        },
        {
            'gz_topic': '/model/wafflepi1/cmd_vel',
            'ros_topic': '/ugv/wafflepi1/cmd_vel',
            'gz_type': 'gz.msgs.Twist',
            'ros_type': 'geometry_msgs/msg/Twist'
        },
    ]

    bridge_arguments = []
    bridge_remappings = []
    for topic in bridge_topics:
        bridge_arguments.append(f'{topic['gz_topic']}@{topic['ros_type']}@{topic['gz_type']}')
        bridge_remappings.append((topic['gz_topic'], topic['ros_topic']))

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='topic_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=bridge_arguments,
        remappings=bridge_remappings
    )

    burger1_republisher_node = Node(
        package='mess2_gazebo',
        executable='ugv_republisher',
        name='republisher',
        output='screen',
        namespace='ugv/burger1',
        parameters=[
            {'tx0': 1.0},
            {'ty0': 0.0},
            {'tz0': 0.0},
            {'rx0': 0.0},
            {'ry0': 0.0},
            {'rz0': 0.0},
            {'rw0': 0.0},
        ]
    )

    wafflepi1_republisher_node = Node(
        package='mess2_gazebo',
        executable='ugv_republisher',
        name='republisher',
        output='screen',
        namespace='ugv/wafflepi1',
        parameters=[
            {'tx0': 2.0},
            {'ty0': -1.0},
            {'tz0': 0.0},
            {'rx0': 0.0},
            {'ry0': 0.0},
            {'rz0': 0.0},
            {'rw0': 0.0},
        ]
    )

    burger1_set_state_action_server_node = Node(
        package='mess2_ugv',
        executable='set_ugv_state_action_server',
        name='set_ugv_state_action_server',
        output='screen',
        namespace='ugv/burger1'
    )

    # burger1_set_state_action_client_node = Node(
    #     package='mess2_ugv',
    #     executable='set_ugv_state_action_client',
    #     name='set_ugv_state_action_client',
    #     output='screen',
    #     namespace='ugv/burger1'
    # )

    ld = LaunchDescription()

    ld.add_action(gz_args)
    ld.add_action(gz_version)
    ld.add_action(ign_args)
    ld.add_action(ign_version)
    ld.add_action(debugger)
    ld.add_action(on_exit_shutdown)

    ld.add_action(OpaqueFunction(function=launch_gz))

    ld.add_action(bridge_node)
    ld.add_action(burger1_republisher_node)
    ld.add_action(wafflepi1_republisher_node)
    ld.add_action(burger1_set_state_action_server_node)
    # ld.add_action(burger1_set_state_action_client_node)

    return ld
