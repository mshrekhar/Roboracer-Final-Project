#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

class ImuCovarianceOverride(Node):
    def __init__(self):
        super().__init__('imu_covariance_override')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(Imu, '/camera/camera/imu', self.cb, qos)
        self.pub = self.create_publisher(Imu, '/imu/data', qos)
        self.gyro_var = 0.001

        self.calibrating = True
        self.cal_samples = []
        self.cal_count = 200
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0
        self.get_logger().info('Calibrating gyro bias — keep car still...')

    def cb(self, msg):
        if self.calibrating:
            self.cal_samples.append((
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ))
            if len(self.cal_samples) >= self.cal_count:
                self.bias_x = sum(s[0] for s in self.cal_samples) / self.cal_count
                self.bias_y = sum(s[1] for s in self.cal_samples) / self.cal_count
                self.bias_z = sum(s[2] for s in self.cal_samples) / self.cal_count
                self.calibrating = False
                self.get_logger().info(
                    f'Gyro bias: x={self.bias_x:.6f} y={self.bias_y:.6f} z={self.bias_z:.6f}'
                )
            # Publish during calibration with current (possibly zero) bias so ICP
            # receives IMU data from frame 1 and never gets a null motion guess.

        gx = msg.angular_velocity.x - self.bias_x
        gy = msg.angular_velocity.y - self.bias_y
        gz = msg.angular_velocity.z - self.bias_z

        msg.angular_velocity.x = gz
        msg.angular_velocity.y = -gx
        msg.angular_velocity.z = -gy

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity_covariance[0] = self.gyro_var
        msg.angular_velocity_covariance[4] = self.gyro_var
        msg.angular_velocity_covariance[8] = self.gyro_var

        msg.header.frame_id = 'base_link'
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuCovarianceOverride())
    rclpy.shutdown()

if __name__ == '__main__':
    main()