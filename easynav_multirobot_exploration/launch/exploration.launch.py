import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg = get_package_share_directory('easynav_multirobot_exploration')

    namespace = LaunchConfiguration('namespace')

    declare_namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for node and topics')

    explorer = Node(
        package='easynav_multirobot_exploration',
        executable='explorer',
        name='explorer',
        parameters=[
            os.path.join(pkg, 'config', 'explorer.params.yaml'),
            {'bt_xml_file': os.path.join(pkg, 'behavior_trees', 'explore.xml')}
            ],
        output='screen',
        namespace=namespace,
        remappings=[
            ('muxed_map', 'maps_manager_node/costmap/incoming_map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_argument)
    ld.add_action(explorer)
    return ld