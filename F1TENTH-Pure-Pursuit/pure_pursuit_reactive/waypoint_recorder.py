import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import numpy as np

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        # Change this to your desired file path
        self.filename = 'waypoints3.csv'
        self.f = open(self.filename, 'w')
        
        # Subscribe to your odometry topic (using your confirmed topic name)
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom', 
            self.odom_callback,
            10)
            
        self.prev_x = None
        self.prev_y = None
        self.min_distance = 0.1  # Record a point every 10cm
        
        self.get_logger().info(f'Recording started. Saving to {self.filename}')
        self.get_logger().info('Drive the car manually to record the path...')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate distance from the last recorded waypoint
        if self.prev_x is not None:
            dist = math.sqrt((x - self.prev_x)**2 + (y - self.prev_y)**2)
        else:
            dist = float('inf')

        # Only record if we've moved enough
        if dist > self.min_distance:
            # Convert Quaternion to Yaw
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # Write to file: x, y, yaw
            self.f.write(f"{x},{y},{yaw}\n")
            self.f.flush()
            
            self.prev_x = x
            self.prev_y = y
            self.get_logger().info(f'Recorded: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

    def __del__(self):
        self.f.close()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Recording stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()