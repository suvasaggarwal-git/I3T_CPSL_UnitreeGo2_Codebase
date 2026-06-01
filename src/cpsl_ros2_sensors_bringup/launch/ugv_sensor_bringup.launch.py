from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node

pkg_livox_ros_driver2 = get_package_share_directory('livox_ros_driver2')

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='namespace'),
    DeclareLaunchArgument('lidar_enable',
                          default_value='true',
                          choices=['true', 'false'],
                          description='Launch the livox lidar'),
    DeclareLaunchArgument('lidar_scan_enable',
                          default_value='true',
                          choices=['true', 'false'],
                          description='If lidar is enabled, additionally publish a /LaserScan message on the /scan topic'),
]

def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration('namespace')
    lidar_enable = LaunchConfiguration('lidar_enable')
    lidar_scan_enable = LaunchConfiguration('lidar_scan_enable')

    namespace_str = namespace.perform(context)
    if namespace_str:
        if not namespace_str.startswith('/'):
            namespace_str = '/' + namespace_str
        tf_prefix = namespace_str.strip("/")
        laser_scan_target_frame = '{}/base_link'.format(tf_prefix)
    else:
        tf_prefix = ""
        laser_scan_target_frame = "base_link"

    launch_livox = PathJoinSubstitution(
        [pkg_livox_ros_driver2, 'launch', 'msg_MID360.launch.py']
    )

    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_livox),
            launch_arguments=[
                ('tf_prefix', tf_prefix)
            ],
            condition=IfCondition(lidar_enable)
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            parameters=[
                {'min_height': 0.5},
                {'max_height': 1.8},
                {'angle_min': -3.141592653589793},
                {'angle_max': 3.141592653589793},
                {'angle_increment': 0.01},
                {'queue_size': 10},
                {'scan_time': 1.0/20.0},
                {'range_min': 0.50},
                {'range_max': 50.0},
                {'target_frame': laser_scan_target_frame},
                {'transform_tolerance': 0.01},
                {'use_inf': False},
            ],
            condition=IfCondition(lidar_scan_enable),
            remappings=[
                ('cloud_in', 'livox/lidar'),
                ('scan', 'livox/scan_best_effort')
            ],
        ),
    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
