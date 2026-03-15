import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('anubi_slam')
    rviz_config = os.path.join(pkg_share, 'config', 'd435i_slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0=None, 1=copy, 2=linear_interpolation'),
        DeclareLaunchArgument(
            'delete_db', default_value='true',
            description='Delete previous map on start (false = resume mapping)'),

        # ── FIX #1 (USB): Increase usbfs memory to reduce control_transfer errors ──
        # This must run before the camera node starts.
        ExecuteProcess(
            cmd=['bash', '-c',
                 'echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb'],
            output='screen',
        ),

        # ── RealSense D435i ──────────────────────────────────────────────────────
        # FIX #1 cont. (USB): Reduced fps from 30→15 to cut USB bandwidth in half.
        # depth_module.profile drives both depth and IR streams; rgb_camera.profile
        # drives the color stream. Lowering both reduces USB congestion on the Orin.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
            launch_arguments={
                'camera_namespace': '',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                'align_depth.enable': 'true',
                'enable_sync': 'true',
                # FIX #1: Reduced from 640x360x30 → 640x360x15
                'rgb_camera.profile': '640x360x15',
                'depth_module.profile': '640x360x15',
            }.items(),
        ),

        # ── IMU fusion ───────────────────────────────────────────────────────────
        # FIX #3 (IMU stall): Added sample_freq
        # to match the RealSense IMU output rate (200 Hz) so the filter does not
        # stall waiting for a magnetometer or mismatched rate.
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
                # FIX #3: Align filter rate to D435i IMU output (200 Hz)
                'sample_freq': 200.0,
                # FIX #3: Slightly larger gain improves convergence at startup
                'gain': 0.01,
            }],
            remappings=[('imu/data_raw', '/camera/imu')],
        ),

        # ── Visual odometry ──────────────────────────────────────────────────────
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'subscribe_odom_info': True,
                'approx_sync': True,
                'wait_imu_to_init': False,
                # FIX #2 (sync): Enlarged queues to tolerate rate/delay differences
                'topic_queue_size': 30,
                'sync_queue_size': 30,
                # FIX #2: Max allowed timestamp gap for approx sync (seconds)
                'approx_sync_max_interval': 0.067,  # ~1 frame at 15 fps

                # Odometry robustness
                'Vis/MinInliers': '10',
                'Vis/MaxFeatures': '600',       # reduced from 1000 → less CPU/USB load
                'Odom/ResetCountdown': '1',
                'OdomF2M/MaxSize': '2000',

                # FIX #2 (VWDictionary): Prevent aggressive word pruning that
                # causes "Not found word XXXXX" errors during live mapping.
                'Kp/MaxFeatures': '400',
                'Mem/RehearsalSimilarity': '0.45',
                'Mem/NotLinkedNodesKept': 'true',
                'Kp/IncrementalFlann': 'true',
            }],
            remappings=[
                ('imu', '/imu/data'),
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ],
        ),

        # ── SLAM ─────────────────────────────────────────────────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=[],
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'subscribe_odom_info': True,
                'approx_sync': True,
                'wait_imu_to_init': False,
                # FIX #2 (sync): Match enlarged queues from odometry node
                'topic_queue_size': 30,
                'sync_queue_size': 30,
                'approx_sync_max_interval': 0.067,

                # FIX #2 (VWDictionary): Same word-management fixes as odom node
                'Vis/MinInliers': '10',
                'Kp/MaxFeatures': '400',
                'Mem/RehearsalSimilarity': '0.45',
                'Mem/NotLinkedNodesKept': 'true',
                'Kp/IncrementalFlann': 'true',
                'Mem/IncrementalMemory': 'true',
                # Relax loop-closure error threshold to reduce false rejections
                'RGBD/OptimizeMaxError': '3.0',

                'Mem/DeleteDbOnStart': LaunchConfiguration('delete_db'),
            }],
            remappings=[
                ('imu', '/imu/data'),
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ],
        ),

        # ── RViz2 ────────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])