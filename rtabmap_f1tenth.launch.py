"""
RTAB-Map SLAM Launch — F1Tenth (v3 — Wall-Slide Fix)
======================================================
ICP-only registration for featureless walls.
Tuned for 1-2 m/s exploration on small tracks.

Key changes from v2:
  - Restored /odom_ekf (IMU-fused) — raw VESC odom drifts too much
  - Switched to point-to-plane ICP — converges better on straight walls
  - Dropped detection rate to 2 Hz — more inter-frame displacement
  - Stricter noise filtering on the output grid
  - Tightened ICP MaxCorrespondenceDistance further (0.08 → 0.06)

Usage:
  ros2 launch mpc slam_launch_v3.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                # 'database_path': '/home/nvidia/.ros/my_map_v3.db',
                'use_sim_time': use_sim_time,

                # ── Topic sync ──────────────────────────────────────────────
                'subscribe_depth': True,
                'subscribe_rgb':   True,
                'subscribe_scan':  True,
                'approx_sync':     True,
                'queue_size':      30,

                # ── Frames ─────────────────────────────────────────────────
                'frame_id':       'base_link',
                'odom_frame_id':  'odom',
                'map_frame_id':   'map',
                'publish_tf':      True,

                # ── Registration: ICP ONLY ─────────────────────────────────
                'Reg/Strategy':   '1',
                'Reg/Force3DoF':  'true',

                # ── ICP tuning ─────────────────────────────────────────────
                # Tightened further — EKF odom gives a good prior, so ICP
                # only needs to refine, not search.
                'Icp/MaxCorrespondenceDistance': '0.06',

                # Point-to-plane: constrains the wall-normal direction
                # accurately while not fighting the unconstrained along-wall
                # direction. Fixes the wall-doubling/smearing pattern.
                'Icp/PointToPlane':              'true',
                'Icp/PointToPlaneK':             '10',
                'Icp/PointToPlaneMinComplexity': '0.02',

                'Icp/Iterations':                '30',
                'Icp/MaxTranslation':            '0.5',
                'Icp/Epsilon':                   '0.001',

                # Coarser voxel — suppress scan noise before ICP.
                'Icp/VoxelSize':                 '0.05',

                # ── ICP outlier rejection ──────────────────────────────────
                'Icp/OutlierRatio':              '0.85',
                'Icp/CorrespondenceRatio':       '0.2',

                # ── Detection rate ─────────────────────────────────────────
                # 2 Hz — larger inter-frame displacement gives ICP more
                # geometric signal on featureless straightaways.
                'Rtabmap/DetectionRate': '2.0',

                # ── Update thresholds ──────────────────────────────────────
                # Fewer graph nodes, each better constrained.
                'RGBD/LinearUpdate':     '0.2',
                'RGBD/AngularUpdate':    '0.15',

                # ── Proximity loop closure ──────────────────────────────────
                'RGBD/ProximityBySpace':          'true',
                'RGBD/ProximityMaxGraphDepth':    '0',
                'RGBD/ProximityPathMaxNeighbors': '10',

                # ── Visual features (loop closure backup) ──────────────────
                'Kp/MaxFeatures': '500',
                'Vis/MinInliers': '8',

                # ── Occupancy grid ─────────────────────────────────────────
                'Grid/FromDepth':  'false',
                'Grid/CellSize':   '0.05',
                'Grid/RangeMin':   '0.2',
                'Grid/RangeMax':   '10.0',
                'Grid/3D':         'false',

                # ── Grid filtering ─────────────────────────────────────────
                'Grid/MaxGroundHeight':            '0.3',
                'Grid/MaxObstacleHeight':          '2.0',
                'Grid/MinClusterSize':             '5',
                'Grid/NoiseFilteringRadius':        '0.1',
                'Grid/NoiseFilteringMinNeighbors':  '5',

                # ── Memory ─────────────────────────────────────────────────
                'Mem/IncrementalMemory':    'true',
                'Mem/InitWMWithAllNodes':   'false',

                # ── Graph optimization ─────────────────────────────────────
                'Optimizer/Strategy':     '1',
                'Optimizer/Iterations':   '100',
                'Optimizer/Robust':       'true',

                # ── Error threshold ────────────────────────────────────────
                'RGBD/OptimizeMaxError':  '1.8',

                'Rtabmap/TimeThr': '0',
            }],
            remappings=[
                ('scan',             '/scan'),
                # ── RESTORED: use EKF-fused odom, not raw VESC ─────────────
                ('odom',             '/odom_ekf'),
                ('rgb/image',        '/camera/camera/infra1/image_rect_raw'),
                ('depth/image',      '/camera/camera/depth/image_rect_raw'),
                ('rgb/camera_info',  '/camera/camera/infra1/camera_info'),
                ('imu',              '/imu/data'),
            ],
        ),
    ])