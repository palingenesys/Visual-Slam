import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

        # ── RealSense D435i ──────────────────────────────────────────
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
                'rgb_camera.profile': '640x360x30',
            }.items(),
        ),

        # ── IMU fusion ───────────────────────────────────────────────
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
            remappings=[('imu/data_raw', '/camera/imu')],
        ),

        # ── Visual odometry ──────────────────────────────────────────
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
                # Lower inlier threshold (default 20 is too strict)
                'Vis/MinInliers': '10',
                # More features to match
                'Vis/MaxFeatures': '1000',
                # Allow odometry reset when lost
                'Odom/ResetCountdown': '1',
                # Use both visual + ICP for robustness
                'OdomF2M/MaxSize': '2000',
            }],
            remappings=[
                ('imu', '/imu/data'),
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ],
        ),

        # ── SLAM ─────────────────────────────────────────────────────
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
                'Vis/MinInliers': '10',
                'Mem/DeleteDbOnStart': LaunchConfiguration('delete_db'),
            }],
            remappings=[
                ('imu', '/imu/data'),
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ],
        ),

        # ── RViz2 ────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
