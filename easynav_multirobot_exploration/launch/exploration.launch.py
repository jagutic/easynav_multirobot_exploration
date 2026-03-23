import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg = get_package_share_directory('easynav_multirobot_exploration')

    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    namespace = LaunchConfiguration('namespace')

    declare_params_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'explorer.params.yaml'),
        description='Absolute path to parameter file'
    )

    declare_bt_xml_argument = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(pkg, 'behavior_trees', 'explore.xml'),
        description='Absolut path to behaviour tree xml file'
    )

    declare_namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for node and topics')

    explorer = Node(
        package='easynav_multirobot_exploration',
        executable='explorer',
        name='explorer',
        parameters=[
            params_file,
            {'bt_xml_file': bt_xml_file}
            ],
        output='screen',
        namespace=namespace,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('frontier_topic', 'maps_manager_node/frontier/points'),
            ('map_topic', 'maps_manager_node/multiplexor/map')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_argument)
    ld.add_action(declare_bt_xml_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(explorer)
    return ld