import os
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def spawn_robots(context):
    """Read the YAML configuration file and spawn the robots defined in it sequentially."""
    config_file = LaunchConfiguration('config').perform(context)

    def convert_floats_to_strings(data):
        """
        Convert all float params in a dict to strings.

        This is required because all launch arguments must be strings.
        """
        if isinstance(data, dict):
            return {k: convert_floats_to_strings(v) for k, v in data.items()}
        elif isinstance(data, list):
            return [convert_floats_to_strings(i) for i in data]
        elif isinstance(data, float):
            return str(data)
        elif isinstance(data, bool):
            return str(data).lower()
        elif data is None:
            return ''  # Convierte campos vacíos de YAML a texto vacío, evitando el NoneType
        else:
            return data

    with open(config_file) as f:
        config_robots = yaml.safe_load(f)

    config_robots = convert_floats_to_strings(config_robots)
    robot_actions = []

    # Sequential spawning with x-second delay between robots to avoid race conditions in Gazebo
    init_delay = 5.0
    spawn_delay = 1.0

    for idx, robot_args in enumerate(config_robots['robots']):
        robot_args['lidar_range'] = '10.0'

        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory('kobuki_description'), 'launch/'),
                    'spawn.launch.py',
                ]
            ),
            launch_arguments=robot_args.items(),
        )

        # Add spawn_delay before each robot spawn (except the first one)
        if idx > 0:
            delayed_spawn = TimerAction(
                period=init_delay + (spawn_delay * idx),
                actions=[spawn_robot],
            )
            robot_actions.append(delayed_spawn)
        else:
            robot_actions.append(spawn_robot)

    # Sequential spawning with 2-second delay between robots to avoid race conditions in Gazebo
    return robot_actions


def generate_launch_description():
    pkg = get_package_share_directory('easynav_multirobot_exploration')

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('maze_world'), 'worlds', 'maze_small.world'
        ),
        description='Simulation world',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=os.path.join(pkg, 'config', 'sim', 'maze_small.params.yaml'),
        description='File for simulated robot parameters',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run gazebo headless',
    )

    # This argument is automatically forwarded to kobuki_description /
    # spawn.launch.py
    declare_do_tf_remapping_arg = DeclareLaunchArgument(
        'do_tf_remapping',
        default_value='true',
        description='Whether to remap the tf topics to independent namespaces (/tf -> tf)',
    )

    # Gz server and client
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s ', world]}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -g ']}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        parameters=[
            {
                'use_sim_time': True,
                'config_file': join(pkg, 'config', 'bridge', 'clock_bridge.yaml'),
            }
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(config_arg)
    ld.add_action(gui_arg)
    ld.add_action(ros_gz_bridge)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(declare_do_tf_remapping_arg)
    ld.add_action(OpaqueFunction(function=spawn_robots))
    return ld
