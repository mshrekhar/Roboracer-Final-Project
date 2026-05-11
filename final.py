#!/usr/bin/env python3
"""
Racing Blind — Unified Pipeline (Adaptive Speed, Pure Pursuit)
==============================================================
Phase 1 (SLAM):   FTG wall-follow builds map via RTAB-Map for N laps
Phase 2 (TRANSITION): Save map → kill SLAM/FTG → restart sensors → launch PF
                       → extract centerline → optimize raceline → launch PP
Phase 3 (RACING): PP starts at start_speed, bumps max_speed by speed_step each
                   lap that tracks well (avg CTE < threshold). When a lap goes
                   OVER threshold, back off to the last good speed and lock it,
                   run one confirmation lap, then stop.

Speed updates go to PP live via set_parameters (no restart).

Usage:
  python3 racing_blind.py --ros-args \\
    -p slam_laps:=1 \\
    -p start_speed:=2.0 \\
    -p speed_step:=0.5 \\
    -p max_speed_limit:=6.0 \\
    -p cte_threshold:=0.15 \\
    -p map_name:=race_track
"""

import math
import time
import os
import subprocess
import signal
import shutil

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class RacingBlind(Node):

    # ── Phases ──
    PHASE_SLAM       = "SLAM"
    PHASE_TRANSITION = "TRANSITION"
    PHASE_RACING     = "RACING"
    PHASE_DONE       = "DONE"

    def __init__(self):
        super().__init__("racing_blind")

        # ══════════════════════════════════════════════════════════
        #  Parameters
        # ══════════════════════════════════════════════════════════
        # SLAM
        self.declare_parameter("slam_laps",       1)

        # Adaptive speed
        self.declare_parameter("start_speed",     2.0)
        self.declare_parameter("speed_step",      0.5)
        self.declare_parameter("max_speed_limit", 6.0)
        self.declare_parameter("min_speed",       1.0)
        self.declare_parameter("cte_threshold",   0.35)   # meters — tune for your track
        self.declare_parameter("max_race_laps",   15)     # safety limit

        # Lap detection
        self.declare_parameter("lap_close_dist",  0.8)
        self.declare_parameter("min_lap_dist",    3.0)
        self.declare_parameter("far_threshold",   1.5)

        # Map / PF / PP
        self.declare_parameter("map_name",        "race_track")
        self.declare_parameter("map_save_path",   "")
        self.declare_parameter("pf_launch_pkg",   "particle_filter")
        self.declare_parameter("pf_launch_file",  "localize_launch.py")
        self.declare_parameter("waypoint_file",   "/home/nvidia/f1tenth_ws/src/centerline.csv")
        self.declare_parameter("smoothing",       2000)

        # Raceline optimization
        self.declare_parameter("optimize_margin", 0.5)   # meters from each wall

        self.slam_laps       = self.get_parameter("slam_laps").get_parameter_value().integer_value
        self.start_speed     = self.get_parameter("start_speed").get_parameter_value().double_value
        self.speed_step      = self.get_parameter("speed_step").get_parameter_value().double_value
        self.max_speed_limit = self.get_parameter("max_speed_limit").get_parameter_value().double_value
        self.min_speed       = self.get_parameter("min_speed").get_parameter_value().double_value
        self.cte_threshold   = self.get_parameter("cte_threshold").get_parameter_value().double_value
        self.max_race_laps   = self.get_parameter("max_race_laps").get_parameter_value().integer_value
        self.lap_close_dist  = self.get_parameter("lap_close_dist").get_parameter_value().double_value
        self.min_lap_dist    = self.get_parameter("min_lap_dist").get_parameter_value().double_value
        self.far_threshold   = self.get_parameter("far_threshold").get_parameter_value().double_value
        self.map_name        = self.get_parameter("map_name").get_parameter_value().string_value
        self.map_save_path   = self.get_parameter("map_save_path").get_parameter_value().string_value
        self.pf_launch_pkg   = self.get_parameter("pf_launch_pkg").get_parameter_value().string_value
        self.pf_launch_file  = self.get_parameter("pf_launch_file").get_parameter_value().string_value
        self.waypoint_file   = self.get_parameter("waypoint_file").get_parameter_value().string_value
        self.smoothing       = self.get_parameter("smoothing").get_parameter_value().integer_value
        self.optimize_margin = self.get_parameter("optimize_margin").get_parameter_value().double_value

        # Auto-detect PF maps folder
        if not self.map_save_path:
            pf_source = os.path.expanduser("~/f1tenth_ws/src/particle_filter/maps")
            if os.path.isdir(pf_source):
                self.map_save_path = os.path.join(pf_source, self.map_name)
            else:
                try:
                    result = subprocess.run(
                        ["ros2", "pkg", "prefix", "particle_filter"],
                        capture_output=True, text=True, timeout=5
                    )
                    pf_share = os.path.join(result.stdout.strip(), "share", "particle_filter", "maps")
                    self.map_save_path = os.path.join(pf_share, self.map_name)
                except Exception:
                    self.map_save_path = os.path.expanduser(
                        f"~/f1tenth_ws/src/particle_filter/maps/{self.map_name}"
                    )

        # ══════════════════════════════════════════════════════════
        #  State
        # ══════════════════════════════════════════════════════════
        self.phase        = self.PHASE_SLAM
        self.lap_count    = 0
        self.lap_times    = []
        self.lap_speeds   = []
        self.lap_ctes     = []
        self.lap_start    = time.time()
        self.total_dist   = 0.0
        self.lap_dist     = 0.0
        self.last_ox      = None
        self.last_oy      = None
        self.was_far      = False
        self.latest_map   = None
        self.pp_process   = None

        # Adaptive speed state
        self.current_max_speed = self.start_speed
        self.last_good_speed   = self.start_speed
        self.speed_locked      = False

        # Waypoints & CTE
        self.waypoints_xy = None
        self.cte_samples  = []

        # ══════════════════════════════════════════════════════════
        #  TF
        # ══════════════════════════════════════════════════════════
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ══════════════════════════════════════════════════════════
        #  Subscribers
        # ══════════════════════════════════════════════════════════
        self.odom_sub = self.create_subscription(Odometry, "/odom_ekf", self.odom_cb, 10)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_cb, map_qos)

        # ══════════════════════════════════════════════════════════
        #  Publishers
        # ══════════════════════════════════════════════════════════
        self.state_pub    = self.create_publisher(String, "/racing_blind/state", 10)
        self.lap_info_pub = self.create_publisher(String, "/racing_blind/lap_info", 10)

        # ══════════════════════════════════════════════════════════
        #  10 Hz Loop
        # ══════════════════════════════════════════════════════════
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  RACING BLIND — Adaptive Speed (Pure Pursuit)")
        self.get_logger().info(f"  SLAM laps:      {self.slam_laps}")
        self.get_logger().info(f"  Start speed:    {self.start_speed} m/s")
        self.get_logger().info(f"  Speed step:     {self.speed_step} m/s")
        self.get_logger().info(f"  Max speed:      {self.max_speed_limit} m/s")
        self.get_logger().info(f"  CTE threshold:  {self.cte_threshold} m")
        self.get_logger().info(f"  Max race laps:  {self.max_race_laps}")
        self.get_logger().info(f"  Optimize margin:{self.optimize_margin} m")
        self.get_logger().info(f"  Map save:       {self.map_save_path}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Waiting for SLAM TF (map → base_link)...")

    # ══════════════════════════════════════════════════════════════
    #  Callbacks
    # ══════════════════════════════════════════════════════════════
    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_ox is not None:
            d = math.hypot(x - self.last_ox, y - self.last_oy)
            self.total_dist += d
            self.lap_dist += d
        self.last_ox = x
        self.last_oy = y

    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    # ══════════════════════════════════════════════════════════════
    #  Waypoints & CTE
    # ══════════════════════════════════════════════════════════════
    def load_waypoints(self, filepath: str):
        try:
            data = np.loadtxt(filepath, delimiter=",")
            if data.ndim == 1:
                data = data[None, :]
            self.waypoints_xy = data[:, :2].astype(float)
            self.get_logger().info(
                f"  Loaded {len(self.waypoints_xy)} waypoints for CTE tracking"
            )
        except Exception as e:
            self.waypoints_xy = None
            self.get_logger().error(f"  Failed to load waypoints for CTE: {e}")

    def compute_cte(self, px: float, py: float) -> float:
        if self.waypoints_xy is None or len(self.waypoints_xy) == 0:
            return 0.0
        dx = self.waypoints_xy[:, 0] - px
        dy = self.waypoints_xy[:, 1] - py
        return float(np.sqrt(np.min(dx * dx + dy * dy)))

    # ══════════════════════════════════════════════════════════════
    #  Main Loop
    # ══════════════════════════════════════════════════════════════
    def loop(self):
        if self.phase == self.PHASE_DONE or self.phase == self.PHASE_TRANSITION:
            return

        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return

        px = t.transform.translation.x
        py = t.transform.translation.y
        dist_to_origin = math.hypot(px, py)
        current_lap_time = time.time() - self.lap_start

        if self.phase == self.PHASE_SLAM:
            target_laps = self.slam_laps
            phase_label = "SLAM"
            speed_str = "FTG"
        else:
            target_laps = self.max_race_laps
            phase_label = "RACE"
            speed_str = f"{self.current_max_speed:.1f}m/s"
            if self.speed_locked:
                speed_str += " [LOCKED]"

        # CTE sampling
        if self.phase == self.PHASE_RACING and self.waypoints_xy is not None:
            self.cte_samples.append(self.compute_cte(px, py))

        avg_cte = float(np.mean(self.cte_samples)) if self.cte_samples else 0.0
        max_cte = float(np.max(self.cte_samples))  if self.cte_samples else 0.0

        # State
        state_msg = String()
        state_msg.data = (
            f"{phase_label} lap {self.lap_count}/{target_laps} | "
            f"speed={speed_str} | "
            f"pos=({px:.2f},{py:.2f}) | "
            f"d_origin={dist_to_origin:.2f}m | "
            f"lap_dist={self.lap_dist:.1f}m | "
            f"time={current_lap_time:.1f}s | "
            f"avg_cte={avg_cte:.3f}m | "
            f"far={self.was_far}"
        )
        self.state_pub.publish(state_msg)

        # Lap info
        parts = []
        for i, lt in enumerate(self.lap_times):
            if i < self.slam_laps:
                parts.append(f"S{i+1}: {lt:.2f}s")
            else:
                ri = i - self.slam_laps
                spd = self.lap_speeds[ri] if ri < len(self.lap_speeds) else 0.0
                cte = self.lap_ctes[ri] if ri < len(self.lap_ctes) else 0.0
                parts.append(f"R{ri+1}: {lt:.2f}s @{spd:.1f}m/s cte={cte:.3f}")
        live_tag = "R" if self.phase == self.PHASE_RACING else "S"
        live_extra = f" cte={avg_cte:.3f}" if self.phase == self.PHASE_RACING else ""
        parts.append(f"{live_tag}{self.lap_count+1}: {current_lap_time:.1f}s (live{live_extra})")
        info_msg = String()
        info_msg.data = " | ".join(parts)
        self.lap_info_pub.publish(info_msg)

        if self.lap_count >= target_laps:
            return

        # Origin gate
        if not self.was_far:
            if dist_to_origin > self.far_threshold:
                self.was_far = True
                self.get_logger().info(f"  [{phase_label}] Left origin | d={dist_to_origin:.1f}m")
            elif self.lap_dist > self.min_lap_dist and dist_to_origin > self.lap_close_dist:
                self.was_far = True
                self.get_logger().info(f"  [{phase_label}] Left origin | lap_dist={self.lap_dist:.1f}m")

        # Lap detection
        if (self.was_far
                and dist_to_origin < self.lap_close_dist
                and self.lap_dist > self.min_lap_dist):

            lap_time = time.time() - self.lap_start
            self.lap_count += 1
            self.lap_times.append(lap_time)

            self.get_logger().info("=" * 60)
            self.get_logger().info(f"  [{phase_label}] LAP {self.lap_count} COMPLETE")
            self.get_logger().info(f"  Time:     {lap_time:.2f} s")
            self.get_logger().info(f"  Distance: {self.lap_dist:.1f} m")
            self.get_logger().info(f"  Avg speed:{self.lap_dist / lap_time:.2f} m/s")

            if self.phase == self.PHASE_SLAM and self.lap_count >= self.slam_laps:
                self.get_logger().info("=" * 60)
                self.get_logger().info("  SLAM PHASE COMPLETE — Starting transition...")
                self.phase = self.PHASE_TRANSITION
                self.transition_to_racing()
                return

            elif self.phase == self.PHASE_RACING:
                self.handle_race_lap_complete(lap_time, avg_cte, max_cte)

            self.get_logger().info("=" * 60)

            self.lap_start   = time.time()
            self.lap_dist    = 0.0
            self.was_far     = False
            self.cte_samples = []

    # ══════════════════════════════════════════════════════════════
    #  Adaptive Speed Decision
    # ══════════════════════════════════════════════════════════════
    def handle_race_lap_complete(self, lap_time: float, avg_cte: float, max_cte: float):
        # Record what this lap was driven at + how it went
        self.lap_speeds.append(self.current_max_speed)
        self.lap_ctes.append(avg_cte)

        self.get_logger().info(f"  Speed:    {self.current_max_speed:.1f} m/s")
        self.get_logger().info(f"  Avg CTE:  {avg_cte:.3f} m (threshold {self.cte_threshold})")
        self.get_logger().info(f"  Max CTE:  {max_cte:.3f} m")

        # Already locked → that was the confirmation lap → done
        if self.speed_locked:
            self.get_logger().info(
                f"  CONFIRMATION LAP at {self.current_max_speed:.1f} m/s — DONE"
            )
            self.print_final_summary()
            self.stop_car()
            self.phase = self.PHASE_DONE
            return

        # Tracked OK → remember this speed as last good, bump up
        if avg_cte < self.cte_threshold:
            self.last_good_speed = self.current_max_speed
            old = self.current_max_speed
            self.current_max_speed = round(
                min(self.current_max_speed + self.speed_step,
                    self.max_speed_limit),
                3,
            )
            self.get_logger().info(
                f"  TRACKING GOOD → speed {old:.1f} → {self.current_max_speed:.1f} m/s"
            )
            self.set_pp_speed(self.current_max_speed)

            # Hit ceiling — lock here, next lap is confirmation
            if self.current_max_speed >= self.max_speed_limit:
                self.get_logger().info(
                    f"  HIT MAX SPEED LIMIT ({self.max_speed_limit:.1f} m/s) — LOCKED"
                )
                self.speed_locked = True

        # Tracking degraded → roll back to last_good_speed and lock
        else:
            old = self.current_max_speed
            self.current_max_speed = self.last_good_speed
            self.speed_locked = True
            self.get_logger().info(
                f"  CTE OVER → backing off {old:.1f} → "
                f"{self.current_max_speed:.1f} m/s [LOCKED]"
            )
            self.set_pp_speed(self.current_max_speed)

        # Safety
        if self.lap_count >= self.max_race_laps:
            self.get_logger().info(f"  MAX RACE LAPS ({self.max_race_laps}) reached")
            self.print_final_summary()
            self.stop_car()
            self.phase = self.PHASE_DONE

    def print_final_summary(self):
        race_times = self.lap_times[self.slam_laps:]
        self.get_logger().info("=" * 60)
        self.get_logger().info("  RACING COMPLETE — FINAL SUMMARY")
        self.get_logger().info(f"  Race laps:     {len(race_times)}")
        self.get_logger().info(f"  Locked speed:  {self.current_max_speed:.1f} m/s")
        self.get_logger().info(f"  Total dist:    {self.total_dist:.1f} m")
        if race_times:
            self.get_logger().info(f"  Best lap:      {min(race_times):.2f}s")
        self.get_logger().info("  ── Lap Details ──")
        for i, (lt, spd, cte) in enumerate(zip(race_times, self.lap_speeds, self.lap_ctes)):
            status = "OK" if cte < self.cte_threshold else "OVER"
            self.get_logger().info(
                f"    R{i+1}: {lt:.2f}s @ {spd:.1f}m/s | avg_cte={cte:.3f}m [{status}]"
            )
        self.get_logger().info("=" * 60)

    # ══════════════════════════════════════════════════════════════
    #  Transition: SLAM → Racing
    # ══════════════════════════════════════════════════════════════
    def transition_to_racing(self):
        # 1. Save map
        if self.latest_map is not None:
            self.save_map(self.latest_map)
        else:
            self.get_logger().error("No /map received — cannot save!")
            self.phase = self.PHASE_DONE
            return

        # 2. Kill SLAM + FTG
        self.get_logger().info("  Killing RTAB-Map + FTG + sensors...")
        for proc in [
                "rtabmap", "rtabmap_slam", "rtabmap_odom", "rtabmap_viz",
                "icp_odometry",
                "component_container", "component_container_isolated", "component_container_mt",
                "sick_bringup", "sensors_bringup",
                "reactive_node", "reactive_wall_follow", "wall_follow",
                "realsense", "realsense2_camera_node",
            ]:
            os.system(f"pkill -f {proc} 2>/dev/null")
        time.sleep(3.0)

        # 3. Restart LiDAR
        self.get_logger().info("  Restarting LiDAR...")
        subprocess.Popen(
            ["ros2", "launch", "f1tenth_stack", "sick_bringup_launch.py"],
            preexec_fn=os.setpgrp
        )
        time.sleep(3.0)

        # 4. Restart Camera + EKF
        self.get_logger().info("  Restarting camera + EKF...")
        subprocess.Popen(
            ["ros2", "launch", "sensors_bringup", "sensors_bringup.launch.py"],
            preexec_fn=os.setpgrp
        )
        time.sleep(5.0)

        # 5. Launch Particle Filter
        self.get_logger().info(f"  Launching PF with map: {self.map_save_path}")
        subprocess.Popen(
            ["ros2", "launch", self.pf_launch_pkg, self.pf_launch_file],
            preexec_fn=os.setpgrp
        )
        time.sleep(5.0)

        # 6. Publish /initialpose at origin
        self.get_logger().info("  Publishing /initialpose at (0, 0, 0)...")
        from geometry_msgs.msg import PoseWithCovarianceStamped
        init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        time.sleep(0.5)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.1

        for _ in range(30):
            msg.header.stamp = self.get_clock().now().to_msg()
            init_pub.publish(msg)
            time.sleep(0.1)

        # 7. Extract centerline
        map_yaml = self.map_save_path + ".yaml"
        centerline_script = os.path.expanduser("~/f1tenth_ws/src/centerline_only.py")
        self.get_logger().info(f"  Extracting centerline (smoothing={self.smoothing}, kappa_max=2.0)...")
        centerline_ok = False
        try:
            result = subprocess.run(
                ["python3", centerline_script, "--map", map_yaml,
                 "--smoothing", str(self.smoothing)],
                cwd=os.path.expanduser("~/f1tenth_ws/src"),
                timeout=60
            )
            if result.returncode == 0:
                self.get_logger().info("  Centerline extraction complete!")
                centerline_ok = True
                shutil.copy2("/home/nvidia/f1tenth_ws/src/centerline.csv",
                             "/home/nvidia/f1tenth_ws/src/actual_centerline.csv")
                self.get_logger().info("  Saved centerline reference: actual_centerline.csv")
            else:
                self.get_logger().error(f"  Centerline extraction failed (exit {result.returncode})")
        except subprocess.TimeoutExpired:
            self.get_logger().error("  Centerline extraction timed out")
        except Exception as e:
            self.get_logger().error(f"  Centerline extraction error: {e}")

        # 7b. Optimize raceline (overwrites centerline.csv with optimized line)
        if centerline_ok:
            optimize_script = os.path.expanduser("~/f1tenth_ws/src/optimize.py")
            self.get_logger().info(
                f"  Optimizing raceline (margin={self.optimize_margin} m)..."
            )
            try:
                result = subprocess.run(
                    ["python3", optimize_script,
                     "--map",        map_yaml,
                     "--centerline", self.waypoint_file,
                     "--margin",     str(self.optimize_margin)],
                    cwd=os.path.expanduser("~/f1tenth_ws/src"),
                    timeout=120,
                )
                if result.returncode == 0:
                    self.get_logger().info("  Raceline optimization complete!")
                else:
                    self.get_logger().error(
                        f"  Raceline optimization failed (exit {result.returncode}) — "
                        f"falling back to centerline"
                    )
            except subprocess.TimeoutExpired:
                self.get_logger().error("  Raceline optimization timed out — falling back to centerline")
            except Exception as e:
                self.get_logger().error(f"  Raceline optimization error: {e} — falling back to centerline")
        else:
            self.get_logger().warn("  Skipping raceline optimization (no centerline)")

        # 8. Load waypoints for CTE (now optimized raceline if step 7b succeeded)
        self.load_waypoints(self.waypoint_file)

        # 9. Launch waypoint visualizer
        viz_script = os.path.expanduser("~/f1tenth_ws/src/frenet_mpc/viz_waypoints.py")
        viz_script1 = os.path.expanduser("~/f1tenth_ws/src/MPC/viz_waypoints.py")

        try:
            subprocess.Popen(
                ["python3", viz_script],
                cwd=os.path.expanduser("~/f1tenth_ws/src"),
                preexec_fn=os.setpgrp
            )
            self.get_logger().info("  Waypoint visualizer launched!")
        except Exception as e:
            self.get_logger().warn(f"  Waypoint visualizer error: {e}")

        try:
            subprocess.Popen(
                ["python3", viz_script1],
                cwd=os.path.expanduser("~/f1tenth_ws/src"),
                preexec_fn=os.setpgrp
            )
            self.get_logger().info("  Waypoint visualizer launched!")
        except Exception as e:
            self.get_logger().warn(f"  Waypoint visualizer error: {e}")

        # 10. Launch Pure Pursuit at start_speed
        self.launch_pp(self.current_max_speed)

        # 11. Enter racing phase
        self.lap_count   = 0
        self.lap_dist    = 0.0
        self.was_far     = False
        self.lap_start   = time.time()
        self.cte_samples = []
        self.phase       = self.PHASE_RACING

        self.get_logger().info("=" * 60)
        self.get_logger().info("  TRANSITION COMPLETE → ADAPTIVE RACING")
        self.get_logger().info(f"  Starting at {self.current_max_speed:.1f} m/s")
        self.get_logger().info(f"  Step:           +{self.speed_step:.1f} m/s on good laps")
        self.get_logger().info(f"  CTE threshold:  {self.cte_threshold} m")
        self.get_logger().info("=" * 60)

    # ══════════════════════════════════════════════════════════════
    #  Map Saving
    # ══════════════════════════════════════════════════════════════
    def save_map(self, grid: OccupancyGrid):
        os.makedirs(os.path.dirname(self.map_save_path), exist_ok=True)

        w   = grid.info.width
        h   = grid.info.height
        res = grid.info.resolution
        ox  = grid.info.origin.position.x
        oy  = grid.info.origin.position.y

        img = np.zeros(w * h, dtype=np.uint8)
        for i, cell in enumerate(grid.data):
            if cell == -1:
                img[i] = 205
            elif cell == 0:
                img[i] = 254
            else:
                img[i] = 0
        img = img.reshape((h, w))
        img = np.flipud(img)

        pgm_path  = self.map_save_path + ".pgm"
        yaml_path = self.map_save_path + ".yaml"

        with open(pgm_path, "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode())
            f.write(img.tobytes())

        with open(yaml_path, "w") as f:
            f.write(f"image: {os.path.basename(pgm_path)}\n")
            f.write(f"resolution: {res}\n")
            f.write(f"origin: [{ox}, {oy}, 0.0]\n")
            f.write(f"negate: 0\n")
            f.write(f"occupied_thresh: 0.65\n")
            f.write(f"free_thresh: 0.196\n")

        self.get_logger().info(f"  Map saved: {pgm_path} ({w}x{h}, res={res}m)")

        copy_targets = [
            os.path.expanduser("~/f1tenth_ws/install/particle_filter/share/particle_filter/maps"),
            os.path.expanduser("~/f1tenth_ws/src/particle_filter/maps"),
        ]
        try:
            result = subprocess.run(
                ["ros2", "pkg", "prefix", "particle_filter"],
                capture_output=True, text=True, timeout=5
            )
            pkg_maps = os.path.join(result.stdout.strip(), "share", "particle_filter", "maps")
            if pkg_maps not in copy_targets:
                copy_targets.append(pkg_maps)
        except Exception:
            pass

        for target_dir in copy_targets:
            try:
                os.makedirs(target_dir, exist_ok=True)
                shutil.copy2(pgm_path, target_dir)
                shutil.copy2(yaml_path, target_dir)
                self.get_logger().info(f"  Map copied to: {target_dir}")
            except Exception as e:
                self.get_logger().warn(f"  Could not copy to {target_dir}: {e}")

    # ══════════════════════════════════════════════════════════════
    #  Pure Pursuit Management
    # ══════════════════════════════════════════════════════════════
    def launch_pp(self, max_speed: float):
        self.get_logger().info(f"  Launching Pure Pursuit (max_speed={max_speed:.1f})...")
        pp_cmd = [
            "ros2", "run", "pure_pursuit_reactive", "pure_pursuit_node",
            "--ros-args",
            "-p", f"waypoint_file:={self.waypoint_file}",
            "-p", f"max_speed:={max_speed}",
            "-p", f"min_speed:={self.min_speed}",
            "-p", "obs_lookahead_time:=0.0",
            "-p", "track_width_threshold:=0.0",
            "-p", "emergency_dist:=0.0",
        ]
        try:
            self.pp_process = subprocess.Popen(pp_cmd, preexec_fn=os.setpgrp)
            self.get_logger().info(f"  PP launched (PID: {self.pp_process.pid})")
        except Exception as e:
            self.get_logger().error(f"  PP launch failed: {e}")

    def set_pp_speed(self, max_speed: float):
        client = self.create_client(
            SetParameters,
            "/pure_pursuit_node/set_parameters"
        )
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("  PP set_parameters service not available!")
            return

        param = Parameter()
        param.name = "max_speed"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = float(max_speed)

        req = SetParameters.Request()
        req.parameters = [param]

        future = client.call_async(req)
        future.add_done_callback(
            lambda f, s=max_speed: self.get_logger().info(
                f"  PP max_speed → {s:.1f} m/s"
            ) if f.result() else self.get_logger().error(
                "  Failed to set PP speed"
            )
        )

    def stop_car(self):
        from ackermann_msgs.msg import AckermannDriveStamped
        pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        time.sleep(0.2)
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        for _ in range(20):
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)
            time.sleep(0.05)
        self.get_logger().info("  Car stopped.")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RacingBlind())
    rclpy.shutdown()


if __name__ == "__main__":
    main()