from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('dog_utilities'),
        'urdf',
        'go2_with_realsense.urdf'
    )

    collect_realsense = LaunchConfiguration('collect_realsense')
    internal_board_ip = LaunchConfiguration('internal_board_ip')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        DeclareLaunchArgument('collect_realsense', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'internal_board_ip',
            default_value='192.168.123.161',
            description='IP of the Go2 control board for WebRTC commands '
                        '(signaling on :9991), reachable over eth0. '
                        'NOT 192.168.1.78 -- that is this machine wlan0.'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}],
            output='screen'
        ),
        Node(
            package='dog_utilities',
            executable='transformUpdater',
            name='dog_transform_updater',
            output='screen'
        ),
        Node(
            package='dog_utilities',
            executable='lidarScanRelay',
            name='lidar_scan_relay',
            output='screen'
        ),
        Node(
            package='dog_utilities',
            executable='cmdVelTranslator',
            name='cmd_vel_translator',
            output='screen',
        ),
        Node(
            package='intel_realsense_functions',
            executable='getCameraFrames',
            name='get_intel_realsense_frames',
            output='screen',
            condition=IfCondition(collect_realsense)
        ),
        Node(
            package='cpsl_nav',
            executable='slamMapPosePub',
            name='slam_map_pose',
            output='screen'
        ),
        Node(
            package='cpsl_detection',
            executable='yoloDetector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.5,
                'dedup_distance_m': 0.5,
            }],
        ),

        # commented out for 3dSLAM, make sure to uncomment this part for 2dSLAM using slam toolbox
        # Node(
        #     package='pointcloud_to_laserscan',
        #     executable='pointcloud_to_laserscan_node',
        #     name='pc_to_scan',
        #     output='screen',
        #     remappings=[
        #         ('cloud_in', '/onboard_lidar_point_cloud2'),
        #         ('scan', '/onboard_lidar_scan')
        #     ],
        #     parameters=[{
        #         'target_frame': '',
        #         'transform_tolerance': 0.01,
        #         'min_height': -0.1,
        #         'max_height': 0.1,
        #         'angle_min': -3.14,
        #         'angle_max': 3.14,
        #         'angle_increment': 0.008,
        #         'scan_time': 0.1,
        #         'range_min': 1.5,
        #         'range_max': 10.0,
        #         'use_inf': True,
        #         'inf_epsilon': 1.0
        #     }]
        # ),
    ])
