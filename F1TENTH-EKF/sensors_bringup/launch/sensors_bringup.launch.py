import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir    = get_package_share_directory('sensors_bringup')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    rs_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py'
    )

    set_ld_library_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value='/opt/ros/humble/lib/aarch64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            'enable_gyro':                'true',
            'enable_accel':               'true',
            'unite_imu_method':           '2',
            'enable_color':               'false',
            'enable_depth':               'true',
            'depth_module.depth_profile': '424x240x30',
            'depth_module.infra_profile': '424x240x30',
            'enable_infra1':              'true',
            'enable_infra2':              'true',
        }.items(),
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0.27', '0', '0.11', '0', '0', '0', 'base_link', 'laser'],
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
    )

    # ── ICP Odometry ─────────────────────────────────────────────────────────
    icp_odom_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id':                      'base_link',
            'odom_frame_id':                 'odom',
            'publish_tf':                    False,
            'approx_sync':                   True,
            'queue_size':                    10,

            # Registration
            'Reg/Strategy':                  '1',      # ICP
            'Reg/Force3DoF':                 'true',   # 2D plane only

            # Odometry strategy
            # 0=F2M (Frame-to-Map): builds local map, robust when single frames fail
            # 1=F2F (Frame-to-Frame): was set, fails when car moves fast
            'Odom/Strategy':                 '0',

            # Primary guess: EKF odom TF (always alive via VESC fallback — survives IMU drops).
            # GuessMotion=true as secondary: used when TF lookup is unavailable at startup.
            # This prevents permanent ICP failure when RealSense restarts its motion module.
            'guess_frame_id':                'odom',
            'Odom/GuessMotion':              'true',
            'Odom/GuessSmoothingDelay':      '0.1',

            # ICP params
            'Icp/MaxTranslation':            '1.0',    # 1m/frame handles 5m/s at 5Hz
            'Icp/MaxRotation':               '1.57',   # 90 deg/frame
            'Icp/VoxelSize':                 '0.05',
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/Iterations':                '50',

            # Reset immediately after any failure so the F2M map re-bootstraps
            # at the current EKF/VESC position rather than staying stale while
            # the robot drives away. Without this one MaxTranslation breach causes
            # permanent loss as the map and robot diverge to 13m+ apart.
            'Odom/ResetCountdown':           '1',

            # Local map size for F2M
            'OdomF2M/MaxSize':               '2000',
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom_icp'),
            ('imu',  '/imu/data'),
        ],
    )

    imu_override_node = Node(
        package='sensors_bringup',
        executable='imu_covariance_override',
        name='imu_covariance_override',
        output='screen',
    )

    # ── EKF ──────────────────────────────────────────────────────────────────
    # Fuses: /odom_icp (LiDAR odometry) + /imu/data (IMU)
    # Publishes: /odom_ekf — used by MPC for velocity, by RTAB-Map for odometry
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'print_diagnostics': True}],
        remappings=[
            ('odometry/filtered', '/odom_ekf'),
        ],
    )

    return LaunchDescription([
        set_ld_library_path,
        laser_tf,
        camera_tf,
        realsense_node,
        imu_override_node,   # start IMU override before ICP needs it
        icp_odom_node,
        ekf_node,
    ])