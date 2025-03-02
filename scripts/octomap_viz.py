import numpy as np
import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header
import os


class OctomapPublisher(Node):
    def __init__(self):
        super().__init__('octomap_publisher')

        self.bt_file_path = '/root/CrazySim/ros2_ws/src/icuas25_competition/worlds/city_1_small/meshes/city_1_small.binvox.bt'

        if not os.path.isfile(self.bt_file_path):
            self.get_logger().error(f"File not found: {self.bt_file_path}")
            rclpy.shutdown()
            return

        with open(self.bt_file_path, 'rb') as file:
            binary_data = file.read()
            self.get_logger().info(f"Read {len(binary_data)} bytes from {self.bt_file_path}")

        # Prepare message once
        self.msg = Octomap()
        self.msg.header = Header()
        self.msg.header.frame_id = "world"
        self.msg.binary = True
        self.msg.id = "OcTree"
        self.msg.resolution = 0.05  # Change if necessary
        signed_data = np.frombuffer(binary_data, dtype=np.int8)
        self.msg.data = signed_data.tolist()

        self.publisher_ = self.create_publisher(Octomap, 'octomap', 10)
        self.timer = self.create_timer(1.0, self.publish_octomap)

        self.get_logger().info(f"Ready to publish OctoMap from {self.bt_file_path}")

    def publish_octomap(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.msg)
        self.get_logger().info("Published Octomap message!")


def main():
    rclpy.init()
    node = OctomapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()