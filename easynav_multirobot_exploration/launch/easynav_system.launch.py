import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_easynav_setup(context, *args, **kwargs):
    sim_config_path = LaunchConfiguration('sim_config_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    navigation_params_file = LaunchConfiguration('navigation_params_file')
    namespace = LaunchConfiguration('namespace')

    with open(sim_config_path) as file:
        sim_config = yaml.safe_load(file)

    overrides = {}
    robot_namespaces = []

    # Extraemos coordenadas y nombres
    for robot in sim_config['robots']:
        ns = robot['namespace']
        robot_namespaces.append(ns)

        prefix = f'multiplexor.{ns}'
        overrides[f'{prefix}.x'] = float(robot['x'])
        overrides[f'{prefix}.y'] = float(robot['y'])
        overrides[f'{prefix}.Y'] = float(robot['Y'])
        overrides[f'{prefix}.topic'] = 'local_map'

    # Añadimos la lista global
    overrides['multiplexor.robot_namespaces'] = robot_namespaces

    easynav_node = Node(
        package='easynav_system',
        executable='system_main',
        namespace=namespace,
        parameters=[navigation_params_file, {'use_sim_time': use_sim_time}, overrides],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )

    return [easynav_node]


def generate_launch_description():
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
    )
    declare_navigation_params_file_cmd = DeclareLaunchArgument(
        'navigation_params_file',
        default_value=os.path.join(
            get_package_share_directory('easynav_multirobot_exploration'),
            'config',
            'navigation_costmap.params.yaml',
        ),
        description='Full path to the ROS2 parameters file to use for the navigation nodes',
    )
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for node and topics'
    )
    declare_sim_config_file_cmd = DeclareLaunchArgument(
        'sim_config_file',
        default_value=os.path.join(
            get_package_share_directory('easynav_multirobot_exploration'),
            'config',
            'sim',
            'maze_small.params.yaml',
        ),
        description='Full path to the simulation YAML config to extract robot coordinates',
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_navigation_params_file_cmd)
    ld.add_action(declare_sim_config_file_cmd)

    ld.add_action(OpaqueFunction(function=launch_easynav_setup))

    return ld
