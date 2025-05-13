#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatteryLogger(Node):
    def __init__(self, num_robots):
        super().__init__('battery_logger')
        self.num_robots = num_robots
        self.battery_status = {i: None for i in range(1, num_robots + 1)}

        for drone_no in range(1, num_robots + 1):
            topic = f"/cf_{drone_no}/battery_status"
            self.create_subscription(
                BatteryState,
                topic,
                lambda msg, drone_no=drone_no: self.battery_callback(msg, drone_no),
                10
            )

        self.create_timer(1.0, self.log_battery)

    def battery_callback(self, msg, drone_no):
        self.battery_status[drone_no] = msg.percentage if msg.percentage >= 0 else None

    def log_battery(self):
        print("\n=== Battery Status ===")
        for drone_no in range(1, self.num_robots + 1):
            percentage = self.battery_status[drone_no]
            if percentage is not None:
                print(f"[Drone {drone_no}] Battery: {percentage:.1f}%")
            else:
                print(f"[Drone {drone_no}] No data yet.")


def main():
    rclpy.init()
    num_robots = int(os.getenv('NUM_ROBOTS', '1'))
    node = BatteryLogger(num_robots)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
