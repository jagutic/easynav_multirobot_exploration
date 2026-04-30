import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap, SetParameter


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    namespace = LaunchConfiguration('namespace')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('easynav_multirobot_exploration'),
            'config',
            'slam_namespaced.params.yaml',
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace for node and topics'
    )

    # Official SLAM Toolbox launcher
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Wrap SLAM Toolbox with namespace
    slam_with_namespace = GroupAction([
        PushRosNamespace(namespace=namespace),

        # Remap for namespace
        SetRemap(src='/map', dst='local_map'),
        SetRemap(src='/map_metadata', dst='local_map_metadata'),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),

        SetParameter(name='map_frame', value=[namespace, '/map']),
        SetParameter(name='odom_frame', value=[namespace, '/odom']),
        SetParameter(name='base_frame', value=[namespace, '/base_footprint']),

        slam_toolbox_launch,
    ])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(slam_with_namespace)

    return ld
