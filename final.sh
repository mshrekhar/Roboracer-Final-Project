#!/bin/bash
# ═══════════════════════════════════════════════════════════════════
#  RACING BLIND — Unified Pipeline Launch (Adaptive Speed)
#  Phase 1: SLAM (RTAB-Map + FTG wall follow) for N laps
#  Phase 2: Auto-transition (save map → PF → centerline → PP)
#  Phase 3: PP ramps speed by speed_step on good laps,
#           locks last good speed when avg CTE > threshold,
#           runs one confirmation lap, then stops.
# ═══════════════════════════════════════════════════════════════════

rm -rf /dev/shm/fastrtps_*

KILL_LIST="wall_follow_node reactive_node wall_follow
rtabmap sick_bringup sick_node ackermann_mux ackermann_to_vesc
vesc_driver_node vesc_to_odom joy_teleop joy_node
static_baselink_to_laser static_transform_publisher
sensors_bringup realsense realsense2_camera_node rs_launch
ekf_filter_node robot_localization icp_odometry
imu_covariance_override particle_filter localize_launch
map_server racing_blind pure_pursuit_node viz_waypoints"

kill_all() {
    for p in $KILL_LIST; do
        pkill -9 -f "$p" 2>/dev/null
    done
}

cleanup() {
    trap - SIGINT SIGTERM
    echo ""
    echo "═══════════════════════════════════════════════════════"
    echo "  KILLING ALL PIPELINE NODES"
    echo "═══════════════════════════════════════════════════════"
    timeout 2 ros2 topic pub --once /drive \
        ackermann_msgs/msg/AckermannDriveStamped \
        "{drive: {speed: 0.0, steering_angle: 0.0}}" 2>/dev/null
    kill_all
    sleep 1
    echo "  Done. Remaining nodes:"
    ros2 node list 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# ═══════════════════════════════════════════════════════════════════
#  CONFIG
# ═══════════════════════════════════════════════════════════════════
SLAM_LAPS=1

# Adaptive speed
START_SPEED=2.0
SPEED_STEP=0.4
MAX_SPEED_LIMIT=6.0
MIN_SPEED=1.0
CTE_THRESHOLD=0.21
MAX_RACE_LAPS=15

# Lap detection
FAR_THRESHOLD=1.5
LAP_CLOSE_DIST=0.8
MIN_LAP_DIST=3.0

# Map / PF / waypoints
MAP_NAME="race_track"
WAYPOINT_FILE="$HOME/f1tenth_ws/src/centerline.csv"
SMOOTHING=2000

TRACKER="$HOME/f1tenth_ws/src/final.py"

# ── Kill mode ──
if [ "$1" = "kill" ]; then
    cleanup
fi

# ── Pre-launch cleanup ──
echo "Cleaning up stale nodes..."
kill_all
sleep 2
echo "  Clean."

echo ""
echo "═══════════════════════════════════════════════════════"
echo "  RACING BLIND — Adaptive Speed Pipeline"
echo "  SLAM:  $SLAM_LAPS laps (wall follow)"
echo "  RACE:  start=$START_SPEED step=+$SPEED_STEP cap=$MAX_SPEED_LIMIT m/s"
echo "         CTE threshold=$CTE_THRESHOLD m, max laps=$MAX_RACE_LAPS"
echo "═══════════════════════════════════════════════════════"
echo ""

# 1. LiDAR
echo "[1/5] Starting LiDAR + f1tenth_stack..."
ros2 launch f1tenth_stack sick_bringup_launch.py &
sleep 3

# 2. Camera + EKF
echo "[2/5] Starting camera + EKF..."
ros2 launch sensors_bringup sensors_bringup.launch.py &
sleep 5

# 3. RTAB-Map SLAM
echo "[3/5] Starting RTAB-Map SLAM..."
rm -f ~/.ros/rtabmap.db
ros2 launch $HOME/f1tenth_ws/src/rtabmap_f1tenth.launch.py &
sleep 5

# 4. FTG (wall follow)
echo "[4/5] Starting wall follow..."
ros2 run wall_follow reactive_node.py &
sleep 1

# 5. Racing Blind (unified tracker)
echo "[5/5] Starting Racing Blind tracker..."
python3 $TRACKER --ros-args \
    -p slam_laps:=$SLAM_LAPS \
    -p start_speed:=$START_SPEED \
    -p speed_step:=$SPEED_STEP \
    -p max_speed_limit:=$MAX_SPEED_LIMIT \
    -p min_speed:=$MIN_SPEED \
    -p cte_threshold:=$CTE_THRESHOLD \
    -p max_race_laps:=$MAX_RACE_LAPS \
    -p far_threshold:=$FAR_THRESHOLD \
    -p lap_close_dist:=$LAP_CLOSE_DIST \
    -p min_lap_dist:=$MIN_LAP_DIST \
    -p map_name:=$MAP_NAME \
    -p waypoint_file:=$WAYPOINT_FILE \
    -p smoothing:=$SMOOTHING &

echo ""
echo "═══════════════════════════════════════════════════════"
echo "  ALL NODES RUNNING — Ctrl+C to kill everything"
echo "  Monitor: ros2 topic echo /racing_blind/state"
echo "  Laps:    ros2 topic echo /racing_blind/lap_info"
echo "═══════════════════════════════════════════════════════"
echo ""

wait