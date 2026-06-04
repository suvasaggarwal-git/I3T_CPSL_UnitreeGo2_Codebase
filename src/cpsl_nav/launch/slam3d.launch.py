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
pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

ARGUMENTS = [
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
                                      'When false, fuses LiDAR geometry with RGB-D color/features.'),
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

    rtabmap_launch_file = PathJoinSubstitution(
        [pkg_rtabmap_launch, 'launch', 'rtabmap.launch.py'])

    rviz_config_file = PathJoinSubstitution(
        [pkg_cpsl_navigation, 'rviz_cfgs', 'slam_3d_config.rviz'])

    if rgbd_only == 'true':
        # Pure RGB-D SLAM: RealSense depth+color drives everything.
        # Reg/Strategy 0 = visual feature matching for loop closure.
        subscribe_scan_cloud = 'false'
        rtabmap_core_args = (
            '--Reg/Strategy 0 '
            '--RGBD/NeighborLinkRefining true '
            '--Vis/MaxDepth 4.0 '
            '--Vis/MinInliers 15 '
            '--Grid/Sensor 1 '
            '--Grid/3D true '
            '--Grid/RangeMax 4.0 '
            '--Grid/MinObstacleHeight 0.1 '
            '--Grid/MaxObstacleHeight 1.5 '
            '--cloud_voxel_size 0.02 '
            '--cloud_decimation 1'
        )
    else:
        # Fused mode: LiDAR provides geometry, RGB-D adds visual loop closure
        # and colors the output cloud_map.
        # Reg/Strategy 2 = ICP + visual features combined.
        subscribe_scan_cloud = 'true'
        rtabmap_core_args = (
            '--Rtabmap/DetectionRate 2.0 '
            '--Reg/Strategy 2 '
            '--RGBD/NeighborLinkRefining true '
            '--Icp/PointToPlane true '
            '--Icp/VoxelSize 0.1 '
            '--Icp/PointToPlaneK 20 '
            '--Vis/MaxDepth 4.0 '
            '--Grid/Sensor 1 '
            '--Grid/3D true '
            '--Grid/RangeMax 10.0 '
            '--Grid/MinObstacleHeight 0.1 '
            '--Grid/MaxObstacleHeight 1.5 '
            '--cloud_voxel_size 0.02 '
            '--cloud_decimation 1'
        )

    if reset_map == 'true':
        rtabmap_core_args += ' --delete_db_on_start'

    slam_3d = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_file),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('frame_id', 'base_link'),
                ('odom_frame_id', 'odom'),
                ('map_frame_id', 'map'),

                # ---- sensor inputs ----
                ('subscribe_scan_cloud', subscribe_scan_cloud),
                ('scan_cloud_topic', scan_cloud_topic),
                ('subscribe_depth', 'true'),
                ('rgb_topic', '/realsense_rgb_image'),
                ('depth_topic', '/realsense_depth_image'),
                ('camera_info_topic', '/depth_camera_intrinsics'),
                # Tell RTAB-Map which TF frame the camera lives in so it can
                # correctly project RGB onto the 3D cloud.
                ('camera_frame_id', 'camera_color_optical_frame'),
                ('rgbd_sync', 'true'),
                ('approx_sync', 'true'),
                ('subscribe_scan', 'false'),

                # ---- odometry: reuse Go2 odom by default ----
                ('visual_odometry', 'false'),
                ('icp_odometry', use_icp_odometry),
                ('odom_topic', odom_topic),
                # DogTransformUpdater owns odom->base_link; don't let RTAB-Map
                # publish a competing transform.
                ('publish_tf_odom', 'false'),

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
                ('odom_args', rtabmap_core_args),
            ],
        ),

        # 2D LiDAR occupancy map via slam_toolbox.
        # Publishes to /slam_toolbox/map to avoid conflicting with RTAB-Map's /map.
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_cpsl_navigation, 'config', 'slam.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('scan', '/livox/scan_best_effort'),
                ('map',  '/slam_toolbox/map'),
            ],
        ),

        # 2D laser scan slice from the RealSense depth image.
        # scan_height=10 averages 10 rows around the image centre (~1m above floor
        # with the camera pitched slightly down).  View on /realsense/scan in RViz.
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
                'decimation': 1,
                'voxel_size': 0.0,
                'approx_sync': True,
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