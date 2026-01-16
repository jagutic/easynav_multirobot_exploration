# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Modified by Juan Carlos Manzanares Serrano

import os
from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import yaml


def spawn_robots(context):
    """Read the YAML configuration file and spawn the robots defined in it."""
    config_file = LaunchConfiguration('robots_config_file').perform(context)

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
        else:
            return data

    config_robots = yaml.safe_load(open(config_file, 'r'))
    config_robots = convert_floats_to_strings(config_robots)

    robot_actions = []
    for robot_args in config_robots['robots']:
        spawn_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('kobuki_description'),
                'launch/'), 'spawn.launch.py']),
            launch_arguments=robot_args.items()
        )
        robot_actions.append(spawn_robot)
    return robot_actions


def generate_launch_description():
    pkg = get_package_share_directory('multirobot_exploration')

    # Diferent simulation enviroments to test
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze',
        description='Tipo de simulación: [maze, hospital]'
    )
    world = LaunchConfiguration('world')

    # Different files for each world
    world_file = PathJoinSubstitution([
        FindPackageShare([ world, "_world" ]),
        'worlds',
        [ world, '.world' ]
    ])
    config_file = PathJoinSubstitution([
        pkg, 'config', 
        [ world, '_sim_params.yaml' ]
    ])

    robots_config_arg = DeclareLaunchArgument(
        'robots_config_file',
        default_value=config_file,
        description='YAML file with the configuration of the robots to be spawned',
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run gazebo headless',
    )

    # This argument is automatically forwarded to kobuki_description / spawn.launch.py
    declare_do_tf_remapping_arg = DeclareLaunchArgument(
        'do_tf_remapping',
        default_value='true',
        description='Whether to remap the tf topics to independent namespaces (/tf -> tf)',
    )


    # Gz server and client
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world_file]}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
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
                'config_file': join(
                    pkg,
                    'config', 'bridge', 'clock_bridge.yaml'
                ),
                'use_sim_time': True,
            }
        ],
        output='screen',
    )

    # start_slam_toolbox = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('slam_toolbox'),
    #         'launch', 
    #         'online_async_launch.py')),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #     }.items()
    # )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(gui_arg)
    ld.add_action(robots_config_arg)
    ld.add_action(declare_do_tf_remapping_arg)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(OpaqueFunction(function=spawn_robots))
    ld.add_action(ros_gz_bridge)

    return ld
