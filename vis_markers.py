import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random

class SphereVisualizer(Node):
    def __init__(self):
        super().__init__("sphere_visualizer")
        self.publisher = self.create_publisher(Marker, "visualization_marker", 10)
        self.timer = self.create_timer(1.0, self.publish_markers)
        self.points = self.generate_points()  # Generate or load your 3D points

    def generate_points(self):
        """ Generate a random set of 3D points """
        return [(random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(-2, 2)) for _ in range(10)]

    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Change this based on your TF setup
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "spheres"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST  # Efficient way to render multiple spheres
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Sphere diameter
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Fully visible
        marker.color.r = 0.0
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0

        # Convert points into ROS geometry_msgs/Point
        for x, y, z in self.points:
            p = Point()
            p.x, p.y, p.z = x, y, z
            marker.points.append(p)

        self.publisher.publish(marker)
        self.get_logger().info("Published marker with {} points".format(len(self.points)))

def main(args=None):
    rclpy.init(args=args)
    node = SphereVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
