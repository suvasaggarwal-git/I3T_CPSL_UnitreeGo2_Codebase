from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node

pkg_cpsl_navigation = get_package_share_directory('cpsl_nav')
pkg_rtabmap_launch = get_package_share_directory('rtabmap_launch')

ARGUMENTS = [
    DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
        [pkg_cpsl_navigation, 'config', 'slam3d.ini']),
        description='RTAB-Map core parameter file'),
    DeclareLaunchArgument('enable_2d_slam', default_value='true',
                          choices=['true', 'false'],
                          description='Also build a separate 2D map (without publishing TF)'),
    DeclareLaunchArgument('approx_sync_max_interval', default_value='0.05',
                          description='Maximum RGB/depth/LiDAR synchronization interval in seconds'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('scan_cloud_topic', default_value='/livox/lidar',
                          description='3D PointCloud2 topic from the Livox driver'),
    DeclareLaunchArgument('odom_topic', default_value='/odom',
                          description='Odometry topic (the Go2 estimator via DogTransformUpdater)'),
    DeclareLaunchArgument('use_icp_odometry', default_value='false',
                          choices=['true', 'false'],
                          description='Compute LiDAR ICP odometry instead of reusing the Go2 odom'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Localize in an existing map instead of mapping'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Open RViz with the 3D SLAM config'),
    DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap_go2.db',
                          description='Where the RTAB-Map database is stored'),
    DeclareLaunchArgument('rgbd_only', default_value='false',
                          choices=['true', 'false'],
                          description='Use RealSense RGB-D only (no LiDAR). '
                                      'When false, maps LiDAR geometry and uses RGB-D features. '
                                      'LiDAR map points are not automatically RGB-colored.'),
    DeclareLaunchArgument('reset_map', default_value='true',
                          choices=['true', 'false'],
                          description='Delete the RTAB-Map database on start so the map is cleared '
                                      'each run. Set false to resume a saved map or use localization.'),
]


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_cloud_topic = LaunchConfiguration('scan_cloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    use_icp_odometry = LaunchConfiguration('use_icp_odometry')
    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    database_path = LaunchConfiguration('database_path')
    rgbd_only = context.perform_substitution(LaunchConfiguration('rgbd_only'))
    reset_map = context.perform_substitution(LaunchConfiguration('reset_map'))
    icp_enabled = context.perform_substitution(use_icp_odometry) == 'true'
    localizing = context.perform_substitution(localization) == 'true'
    if icp_enabled and rgbd_only == 'true':
        raise RuntimeError('use_icp_odometry requires rgbd_only:=false')

    rtabmap_launch_file = PathJoinSubstitution(
        [pkg_rtabmap_launch, 'launch', 'rtabmap.launch.py'])

    rviz_config_file = PathJoinSubstitution(
        [pkg_cpsl_navigation, 'rviz_cfgs', 'slam_3d_config.rviz'])

    if rgbd_only == 'true':
        # Pure RGB-D SLAM: RealSense depth+color drives everything.
        # Reg/Strategy 0 = visual feature matching for loop closure.
        subscribe_scan_cloud = 'false'
        rtabmap_core_args = '--Reg/Strategy 0 --Grid/Sensor 1'
    else:
        # LiDAR map geometry, with RGB-D features for registration/loop closure.
        # Reg/Strategy 1 = ICP on the LiDAR clouds (RGB-D features still detect
        # loop closures).
        subscribe_scan_cloud = 'true'
        rtabmap_core_args = '--Reg/Strategy 1 --Grid/Sensor 0'

    if reset_map == 'true' and not localizing:
        rtabmap_core_args += ' --delete_db_on_start'

    slam_3d = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('cfg', LaunchConfiguration('config')),
                ('frame_id', 'base_link'),
                # Use external TF by default. ICP mode must consume its own odom
                # messages instead of silently continuing to use Go2 TF.
                ('odom_frame_id', '' if icp_enabled else 'odom'),
                ('vo_frame_id', 'odom'),
                ('map_frame_id', 'map'),

                # ---- sensor inputs ----
                ('subscribe_scan_cloud', subscribe_scan_cloud),
                ('scan_cloud_topic', scan_cloud_topic),
                ('depth', 'true'),
                ('subscribe_rgbd', 'true'),
                ('rgb_topic', '/realsense_rgb_image'),
                ('depth_topic', '/realsense_depth_image'),
                ('camera_info_topic', '/depth_camera_intrinsics'),
                ('rgbd_sync', 'true'),
                ('approx_rgbd_sync', 'true'),
                ('approx_sync', 'true'),
                ('approx_sync_max_interval', LaunchConfiguration('approx_sync_max_interval')),
                ('subscribe_scan', 'false'),

                # ---- odometry: reuse Go2 odom by default ----
                ('visual_odometry', 'false'),
                ('icp_odometry', use_icp_odometry),
                ('odom_topic', odom_topic),
                # In ICP mode, launch dog.launch.py with publish_odom_tf:=false.
                ('publish_tf_odom', 'true' if icp_enabled else 'false'),
                ('publish_tf_map', 'true'),

                # ---- timing / QoS ----
                ('qos', '2'),
                ('qos_scan', '2'),
                ('wait_for_transform', '0.2'),

                # ---- mapping vs localization ----
                ('localization', localization),
                ('database_path', database_path),

                # ---- viz handled separately below ----
                ('rtabmapviz', 'false'),
                ('rviz', 'false'),

                ('rtabmap_args', rtabmap_core_args),
            ],
        ),

        # 2D LiDAR occupancy map via slam_toolbox.
        # Topic remapping alone does not prevent map->odom TF conflicts.
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_2d_slam')),
            parameters=[
                PathJoinSubstitution([pkg_cpsl_navigation, 'config', 'slam.yaml']),
                {'use_sim_time': use_sim_time, 'transform_publish_period': 0.0},
            ],
            remappings=[
                ('scan', '/livox/scan_best_effort'),
                ('map',  '/slam_toolbox/map'),
            ],
        ),

        # 2D laser scan slice from the RealSense depth image.
        # scan_height selects rows around the image centre; the converter takes
        # the nearest valid depth in each column. View /realsense/scan in RViz.
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='realsense_to_laserscan',
            output='screen',
            parameters=[{
                'scan_height': 10,
                'range_min': 0.2,
                'range_max': 4.0,
                'output_frame': 'camera_color_optical_frame',
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('depth',             '/realsense_depth_image'),
                ('depth_camera_info', '/depth_camera_intrinsics'),
                ('scan',              '/realsense/scan'),
            ],
        ),

        # Live colored point cloud from RealSense depth + RGB.
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyzrgb',
            name='realsense_color_cloud',
            output='screen',
            parameters=[{
                'decimation': 2,
                'voxel_size': 0.05,
                'min_depth': 0.3,
                'max_depth': 3.5,
                'approx_sync': True,
                'approx_sync_max_interval': LaunchConfiguration('approx_sync_max_interval'),
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('rgb/image',       '/realsense_rgb_image'),
                ('depth/image',     '/realsense_depth_image'),
                ('rgb/camera_info', '/depth_camera_intrinsics'),
                ('cloud',           '/realsense/depth/color/points'),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])

    return [slam_3d]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
