"""
SLAM launch for Unitree Go2 deployment with DDS odometry publishing.

Key differences from slam.launch.py:
  - Adds static TF: base_link → camera_link (configurable mount offset)
  - RViz2 is kept and runs in parallel; disable with rviz:=false for pure headless mode
  - USB-memory tweak moved to docker-entrypoint.sh (runs before container starts nodes)
  - use_slam:=false for pure VIO odometry without RTAB-Map backend

Odometry published to DDS network:
  /odom               nav_msgs/Odometry   — VIO camera odometry at ~15 Hz
  /tf                 tf2_msgs/TFMessage  — odom→camera_link, base_link→camera_link

Remote RViz2 (laptop):
  Set ROS_DOMAIN_ID and RMW_IMPLEMENTATION=rmw_cyclonedds_cpp on the remote machine.
  The /rtabmap/mapData, /rtabmap/mapGraph, /rtabmap/map topics are discoverable via DDS.
  Then run:  ros2 launch anubi_slam slam_go2.launch.py rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('anubi_slam')
    rviz_config = os.path.join(pkg_share, 'config', 'd435i_slam.rviz')

    return LaunchDescription([

        # ── Launch arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='IMU unification method: 0=None, 1=copy, 2=linear_interpolation'),
        DeclareLaunchArgument(
            'delete_db', default_value='true',
            description='Delete previous map on start (false = resume mapping)'),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='Launch RTAB-Map backend (loop-closure + map). '
                        'Set false for pure VIO odometry only.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 for live map visualization. '
                        'Set false for headless robot-side deploy; '
                        'run RViz2 remotely via DDS instead.'),

        # Camera mount offset from base_link (Go2 body frame) ─────────────
        # Measure the physical offset of the D435i from Go2 base_link origin.
        DeclareLaunchArgument('cam_x',     default_value='0.1',  description='Camera X offset from base_link (m)'),
        DeclareLaunchArgument('cam_y',     default_value='0.0',  description='Camera Y offset from base_link (m)'),
        DeclareLaunchArgument('cam_z',     default_value='0.15', description='Camera Z offset from base_link (m)'),
        DeclareLaunchArgument('cam_roll',  default_value='0.0',  description='Camera roll  (rad)'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0',  description='Camera pitch (rad)'),
        DeclareLaunchArgument('cam_yaw',   default_value='0.0',  description='Camera yaw   (rad)'),

        # ── RealSense D435i ──────────────────────────────────────────────────
        # 640×360@15fps — halved bandwidth vs 30fps to stay within USB 3.0 budget
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch', 'rs_launch.py')]),
            launch_arguments={
                'camera_namespace':   '',
                'enable_gyro':        'true',
                'enable_accel':       'true',
                'unite_imu_method':   LaunchConfiguration('unite_imu_method'),
                'align_depth.enable': 'true',
                'enable_sync':        'true',
                'rgb_camera.profile':   '640x360x15',
                'depth_module.profile': '640x360x15',
            }.items(),
        ),

        # ── Static TF: base_link → camera_link ──────────────────────────────
        # Required so the nav stack and Go2 SDK can express odometry in base_link.
        # Adjust cam_* args to match the physical mount position on your Go2.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            output='screen',
            arguments=[
                LaunchConfiguration('cam_x'),
                LaunchConfiguration('cam_y'),
                LaunchConfiguration('cam_z'),
                LaunchConfiguration('cam_yaw'),
                LaunchConfiguration('cam_pitch'),
                LaunchConfiguration('cam_roll'),
                'base_link',
                'camera_link',
            ],
        ),

        # ── IMU fusion (Madgwick) ────────────────────────────────────────────
        # sample_freq=200 matches D435i native IMU output rate; prevents stall.
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag':     False,
                'world_frame': 'enu',
                'publish_tf':  False,
                'sample_freq': 200.0,
                'gain': 0.01,
            }],
            remappings=[('imu/data_raw', '/camera/imu')],
        ),

        # ── Visual-Inertial Odometry ─────────────────────────────────────────
        # Publishes:
        #   /odom           nav_msgs/Odometry   — real-time VIO pose at ~15 Hz
        #   /tf             odom → camera_link
        #   /odom_info      rtabmap_msgs/OdomInfo
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth':    True,
                'subscribe_odom_info': True,
                'approx_sync': True,
                'wait_imu_to_init': False,
                'topic_queue_size': 30,
                'sync_queue_size':  30,
                'approx_sync_max_interval': 0.067,  # ~1 frame @ 15fps

                'Vis/MinInliers':  '10',
                'Vis/MaxFeatures': '600',
                'Odom/ResetCountdown': '1',
                'OdomF2M/MaxSize': '2000',

                'Kp/MaxFeatures':          '400',
                'Mem/RehearsalSimilarity': '0.45',
                'Mem/NotLinkedNodesKept':  'true',
                'Kp/IncrementalFlann':     'true',
            }],
            remappings=[
                ('imu',             '/imu/data'),
                ('rgb/image',       '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),
            ],
        ),

        # ── SLAM backend (optional) ──────────────────────────────────────────
        # Provides loop-closure, pose-graph optimisation, and 3D map.
        # Disable with use_slam:=false if you only need /odom (saves CPU).
        Node(
            condition=IfCondition(LaunchConfiguration('use_slam')),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth':     True,
                'subscribe_odom_info': True,
                'approx_sync': True,
                'wait_imu_to_init': False,
                'topic_queue_size': 30,
                'sync_queue_size':  30,
                'approx_sync_max_interval': 0.067,

                'Vis/MinInliers':          '10',
                'Kp/MaxFeatures':          '400',
                'Mem/RehearsalSimilarity': '0.45',
                'Mem/NotLinkedNodesKept':  'true',
                'Kp/IncrementalFlann':     'true',
                'Mem/IncrementalMemory':   'true',
                'RGBD/OptimizeMaxError':   '3.0',

                'Mem/DeleteDbOnStart': LaunchConfiguration('delete_db'),
            }],
            remappings=[
                ('imu',             '/imu/data'),
                ('rgb/image',       '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image',     '/camera/aligned_depth_to_color/image_raw'),
            ],
        ),

        # ── RViz2 ────────────────────────────────────────────────────────────
        # Runs in parallel with all publishing nodes.
        # On the robot: set rviz:=false and connect a remote RViz2 via DDS.
        # With X11 forwarding (ssh -X): leave rviz:=true.
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
