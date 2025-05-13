#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class OdomLogger(Node):
    def __init__(self, num_robots):
        super().__init__('odom_logger')
        self.num_robots = num_robots
        self.poses = {i: None for i in range(1, num_robots + 1)}

        for drone_no in range(1, num_robots + 1):
            topic = f"/cf_{drone_no}/pose"
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, drone_no=drone_no: self.pose_callback(msg, drone_no),
                10
            )

        self.create_timer(1.0, self.log_poses)

    def pose_callback(self, msg, drone_no):
        self.poses[drone_no] = msg

    def log_poses(self):
        print("\n=== Odometry Data ===")
        for drone_no in range(1, self.num_robots + 1):
            pose = self.poses[drone_no]
            if pose:
                pos = pose.pose.position
                print(f"[Drone {drone_no}] x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f}")
            else:
                print(f"[Drone {drone_no}] No data yet.")


def main():
    rclpy.init()
    num_robots = int(os.getenv('NUM_ROBOTS', '1'))
    node = OdomLogger(num_robots)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
