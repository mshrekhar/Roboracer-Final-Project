#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray


class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow_node')

        self.declare_parameter('desired_distance',   0.8)
        self.declare_parameter('lookahead_distance', 3.0)
        self.declare_parameter('theta_deg',          50.0)

        self.declare_parameter('Kp',                 2.0)
        self.declare_parameter('Kd',                 0.04)
        self.declare_parameter('Ki',                 0.0)

        self.declare_parameter('max_speed',          1.0)
        self.declare_parameter('min_speed',          1.0)

        self.declare_parameter('ema_alpha',          0.3)
        self.declare_parameter('deadband',           0.6)
        self.declare_parameter('deadband_shrink',    0.8)
        self.declare_parameter('integral_clamp',     1.0)
        self.declare_parameter('steering_clamp',     0.8)
        self.declare_parameter('right_bias',         0.0)  # positive = nudge away from wall

        self.desired_distance   = self.get_parameter('desired_distance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.theta              = np.radians(self.get_parameter('theta_deg').value)

        self.Kp              = self.get_parameter('Kp').value
        self.Kd              = self.get_parameter('Kd').value
        self.Ki              = self.get_parameter('Ki').value

        self.max_speed       = self.get_parameter('max_speed').value
        self.min_speed       = self.get_parameter('min_speed').value

        self.ema_alpha       = self.get_parameter('ema_alpha').value
        self.deadband        = self.get_parameter('deadband').value
        self.deadband_shrink = self.get_parameter('deadband_shrink').value
        self.integral_clamp  = self.get_parameter('integral_clamp').value
        self.steering_clamp  = self.get_parameter('steering_clamp').value
        self.right_bias      = self.get_parameter('right_bias').value

        self.integral          = 0.0
        self.prev_error        = 0.0
        self.filtered_steering = 0.0
        self.pid_initialized   = False
        self.prev_time         = self.get_clock().now()

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.diag_pub = self.create_publisher(
            Float32MultiArray, '/wall_follow_diag', 10)

        self.get_logger().info(
            f"WallFollow (RIGHT) | Kp={self.Kp} Kd={self.Kd} Ki={self.Ki} "
            f"alpha={self.ema_alpha} deadband={self.deadband} "
            f"speed=[{self.min_speed},{self.max_speed}]"
        )

    def get_range(self, msg: LaserScan, angle: float) -> float:
        angle = np.clip(angle, msg.angle_min, msg.angle_max)
        idx   = int((angle - msg.angle_min) / msg.angle_increment)
        idx   = np.clip(idx, 0, len(msg.ranges) - 1)
        r     = msg.ranges[idx]
        return msg.range_max if (np.isnan(r) or np.isinf(r)) else float(r)

    def get_error(self, msg: LaserScan) -> tuple[float, float, float]:
        # ── Right-wall beams are negative angles ──
        # b points directly right (-90°), a is theta ahead of that
        b_angle = -np.pi / 2.0                  # directly right
        a_angle = -np.pi / 2.0 + self.theta     # rotated forward by theta.


        a = self.get_range(msg, a_angle)
        b = self.get_range(msg, b_angle)

        alpha = np.arctan2(a * np.cos(self.theta) - b,
                           a * np.sin(self.theta))
        Dt  = b * np.cos(alpha)
        Dt1 = Dt + self.lookahead_distance * np.sin(alpha)

        # Error: positive means too far from wall (steer right), negative means too close (steer left)
        error = self.desired_distance - Dt1 + self.right_bias

        return error, Dt, alpha

    def speed_from_steering(self, abs_steer: float) -> float:
        if   abs_steer < 0.10: return self.max_speed
        elif abs_steer < 0.30: return self.max_speed * 0.75
        elif abs_steer < 0.50: return self.min_speed
        else:                  return self.min_speed * 0.70

    def scan_callback(self, msg: LaserScan):
        error, Dt, alpha = self.get_error(msg)

        if abs(error) < self.deadband:
            error *= (1.0 - self.deadband_shrink)

        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        dt  = max(dt, 1e-6)

        self.integral += error * dt
        self.integral  = np.clip(self.integral, -self.integral_clamp, self.integral_clamp)

        if self.pid_initialized:
            derivative = (error - self.prev_error) / dt
        else:
            derivative           = 0.0
            self.pid_initialized = True
        self.prev_error = error

        raw_steering = (self.Kp * error
                        + self.Ki * self.integral
                        + self.Kd * derivative)
        raw_steering = float(np.clip(raw_steering, -self.steering_clamp, self.steering_clamp))

        self.filtered_steering = (self.ema_alpha * self.filtered_steering
                                  + (1.0 - self.ema_alpha) * raw_steering)

        speed = self.speed_from_steering(abs(self.filtered_steering))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp         = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = self.filtered_steering
        drive_msg.drive.speed          = speed
        self.drive_pub.publish(drive_msg)

        diag = Float32MultiArray()
        diag.data = [
            float(error),
            float(raw_steering),
            float(self.filtered_steering),
            float(Dt),
            float(alpha),
            float(speed),
            float(dt),
            float(self.integral),
        ]
        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()