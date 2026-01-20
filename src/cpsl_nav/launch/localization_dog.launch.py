import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import PushRosNamespace, SetRemap, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

pkg_cpsl_navigation = get_package_share_directory('cpsl_nav')

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('scan_topic', default_value='/scan',
                          description='LaserScan topic to use for slam_toolbox'),
    DeclareLaunchArgument('slam_params_file',
                          default_value='mapper_params_localization_dog.yaml',
                          description='YAML file in cpsl_nav/config for slam_toolbox'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Auto configure/activate lifecycle node'),
    DeclareLaunchArgument('use_lifecycle_manager', default_value='false',
                          choices=['true', 'false'],
                          description='If true, expect an external lifecycle manager'),
]

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    # Resolve strings for remaps
    namespace_str = namespace.perform(context)
    if namespace_str and not namespace_str.startswith('/'):
        namespace_str = '/' + namespace_str
    scan_topic_str = scan_topic.perform(context)

    # Resolve params file under cpsl_nav/config
    slam_params_file_str = slam_params_file.perform(context)
    param_file = PathJoinSubstitution([pkg_cpsl_navigation, 'config', slam_params_file_str])

    # Lifecycle node
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            param_file,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': scan_topic,                # also set param explicitly
                'use_lifecycle_manager': use_lifecycle_manager,
            },
        ],
    )

    # autostart AND (not use_lifecycle_manager)
    autostart_condition = IfCondition(PythonExpression([
        "'", autostart, "'", " == 'true' and '", use_lifecycle_manager, "' != 'true'"
    ]))

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=autostart_condition
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] slam_toolbox node is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=autostart_condition
    )

    # Group for namespace + remaps (mirrors your working style)
    localization_group = GroupAction([
        PushRosNamespace(namespace),

        # Remap common topics into the namespace
        SetRemap('/tf', namespace_str + '/tf'),
        SetRemap('/tf_static', namespace_str + '/tf_static'),
        SetRemap(namespace_str + '/scan', namespace_str + scan_topic_str),
        SetRemap('/map', namespace_str + '/map'),

        slam_node,
        configure_event,
        activate_event,
    ])

    return [localization_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
