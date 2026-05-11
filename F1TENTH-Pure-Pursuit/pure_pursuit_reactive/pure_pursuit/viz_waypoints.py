import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import csv

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer')
        self.publisher = self.create_publisher(Marker, '/pure_pursuit/path_viz', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)
        self.waypoints_path = '/home/nvidia/f1tenth_ws/src/lab-5-slam-and-pure-pursuit-team8/pure_pursuit/waypoints.csv'

    def publish_waypoints(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Line width
        marker.scale.x = 0.05 
        # Color: Blue
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0

        try:
            with open(self.waypoints_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row or row[0].startswith('#'): continue
                    p = Point()
                    p.x = float(row[0])
                    p.y = float(row[1])
                    p.z = 0.05 # Slightly above ground to avoid flickering
                    marker.points.append(p)
            
            self.publisher.publish(marker)
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV: {e}")

def main():
    rclpy.init()
    node = WaypointVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()