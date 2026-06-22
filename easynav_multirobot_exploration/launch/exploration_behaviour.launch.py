import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg = get_package_share_directory('easynav_multirobot_exploration')

    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    namespace = LaunchConfiguration('namespace')

    save_data = LaunchConfiguration('save_data')
    data_path = LaunchConfiguration('data_path')

    declare_params_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'explorer.params.yaml'),
        description='Absolute path to parameter file',
    )

    declare_bt_xml_argument = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(pkg, 'behavior_trees', 'explore.xml'),
        description='Absolut path to behaviour tree xml file',
    )

    declare_namespace_argument = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for node and topics'
    )

    declare_save_map_argument = DeclareLaunchArgument(
        'save_data', default_value='false', description='Whether to save map data'
    )

    declare_save_map_path_argument = DeclareLaunchArgument(
        'data_path',
        default_value=os.path.expanduser('~/Documentos/Uni/TFG/TFG_ws/src/easynav_multirobot_exploration/easynav_multirobot_exploration/data/'),
        description='Path to save map data CSV'
    )

    explorer = Node(
        package='easynav_multirobot_exploration',
        executable='explorer',
        name='explorer',
        parameters=[params_file, {'bt_xml_file': bt_xml_file}],
        output='screen',
        namespace=namespace,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('frontier_topic', 'maps_manager_node/frontier/points'),
        ],
    )

    save_map_data_node = Node(
        condition=IfCondition(save_data),
        package='easynav_multirobot_exploration',
        executable='save_map_data',
        name='save_map_data',
        namespace=namespace,
        parameters=[{
            'csv_path': data_path,
        }],
        remappings=[
            ('map_topic', 'maps_manager_node/multiplexor/map'),
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_argument)
    ld.add_action(declare_bt_xml_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_save_map_argument)
    ld.add_action(declare_save_map_path_argument)
    ld.add_action(explorer)
    ld.add_action(save_map_data_node)
    return ld
