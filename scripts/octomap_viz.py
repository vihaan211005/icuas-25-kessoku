import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import os
import numpy as np
import octomap
import pcl  # Requires python-pcl for point cloud conversion
from pcl import PointCloud


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

        # Load OctoMap
        self.octree = octomap.OcTree(0.05)  # Adjust resolution
        self.octree.readBinary(self.bt_file_path)

        self.publisher_octomap = self.create_publisher(Octomap, 'octomap', 10)
        self.publisher_pointcloud = self.create_publisher(PointCloud2, 'octomap_pointcloud', 10)

        self.timer = self.create_timer(1.0, self.publish_octomap)

        self.get_logger().info(f"Ready to publish OctoMap from {self.bt_file_path}")

    def publish_octomap(self):
        msg = Octomap()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.binary = True
        msg.id = "OcTree"
        msg.resolution = 0.05
        signed_data = np.frombuffer(self.octree.writeBinary(), dtype=np.int8)
        msg.data = signed_data.tolist()

        self.publisher_octomap.publish(msg)
        self.get_logger().info("Published Octomap message!")

        # Convert OctoMap to PointCloud2
        pointcloud_msg = self.octomap_to_pointcloud()
        if pointcloud_msg:
            self.publisher_pointcloud.publish(pointcloud_msg)
            self.get_logger().info("Published Octomap PointCloud2!")

    def octomap_to_pointcloud(self):
        """ Convert OctoMap to a PointCloud2 message """
        points = []
        for node in self.octree.begin_leafs():
            if self.octree.isNodeOccupied(node):
                x, y, z = node.getCoordinate()
                points.append([x, y, z])

        if not points:
            self.get_logger().warning("No occupied nodes found in OctoMap!")
            return None

        # Convert to PCL PointCloud
        cloud = PointCloud()
        cloud.from_array(np.array(points, dtype=np.float32))

        # Convert to ROS2 PointCloud2 message
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header.frame_id = "map"
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points)
        pointcloud_msg.is_dense = True

        # Set fields
        pointcloud_msg.fields = [
            dict(name='x', offset=0, datatype=7, count=1),
            dict(name='y', offset=4, datatype=7, count=1),
            dict(name='z', offset=8, datatype=7, count=1),
        ]
        pointcloud_msg.point_step = 12
        pointcloud_msg.row_step = pointcloud_msg.point_step * len(points)
        pointcloud_msg.data = np.array(points, dtype=np.float32).tobytes()

        return pointcloud_msg


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