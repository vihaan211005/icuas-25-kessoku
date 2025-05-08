#!/usr/bin/env python3
import socket
import struct
import cv2
import numpy as np
import time
from typing import Dict

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

CPX_HEADER_SIZE = 4
IMG_HEADER_MAGIC = 0xBC
IMG_HEADER_SIZE = 11
UDP_PORT = 5001

MAGIC_BYTE = b'FER'  # magic byte sent to aideck to identify us as client
MAGIC_TIMEOUT = 2.0  # seconds

class MagicSender:
    """Responsible for sending magic data to aideck, so it can know what ip to send data to."""
    def __init__(self, node, sock: socket.socket, ip_to_cf: Dict[str, str]):
        self.node = node
        self.sock = sock
        self.ip_to_cf = ip_to_cf
        self.last_seen = {}

        # Send initial magic packets
        for ip in ip_to_cf:
            self.send_magic(ip)

    def update_last_seen(self, ip):
        self.last_seen[ip] = time.time()

    def check_and_send_magic(self):
        now = time.time()
        for ip in self.ip_to_cf:
            last = self.last_seen.get(ip, 0)
            if now - last > MAGIC_TIMEOUT:
                self.node.get_logger().info(f"[{ip}] No image recently. Sending magic byte.")
                self.send_magic(ip)
                self.last_seen[ip] = now

    def send_magic(self, ip):
        try:
            self.sock.sendto(MAGIC_BYTE, (ip, 5000))  # ESP32 listens on 5000
            self.node.get_logger().info(f" Sent magic byte to {ip}:5000")
        except Exception as e:
            self.node.get_logger().warn(f" Failed to send magic to {ip}: {e}")


class PerAddressStream:
    """Streamer class for each aideck"""
    def __init__(self, node: Node, addr: tuple, bridge: CvBridge, cf_id: str, camera_info_config: dict = None):
        ip, port = addr
        self.addr = addr
        self.node = node
        self.bridge = bridge
        self.cf_id = cf_id if cf_id is not None else f"host_{ip.replace('.', '_')}"
        self.buffer = bytearray()
        self.expected_size = None
        self.receiving = False
        self.last_frame_time = None
        self.window_name = f"Stream from {self.cf_id}"

        # Topics
        self.image_topic = f"/{self.cf_id}/image"
        self.caminfo_topic = f"/{self.cf_id}/camera_info"

        # Publishers
        self.image_pub = node.create_publisher(Image, self.image_topic, 10)
        self.caminfo_pub = node.create_publisher(CameraInfo, self.caminfo_topic, 10) if camera_info_config else None

        # Static camera info message
        self.camera_info_msg = self._construct_camera_info(camera_info_config) if camera_info_config else None

        self.node.get_logger().info(f"[{self.window_name}] Created publishers on {self.image_topic}")


    def _construct_camera_info(self, config):
        msg = CameraInfo()
        msg.width = int(config['image_width'])
        msg.height = int(config['image_height'])
        msg.distortion_model = config['distortion_model']
        msg.d = config['distortion_coefficients']['data']
        msg.k = config['camera_matrix']['data']
        msg.r = config['rectification_matrix']['data']
        msg.p = config['projection_matrix']['data']
        msg.header.frame_id = f"{self.cf_id}/camera"
        return msg


    def update_header(self, width, height, depth, fmt, size, payload_rest):
        self.expected_size = size
        self.buffer = bytearray(payload_rest)
        self.receiving = True


    def append_data(self, chunk):
        self.buffer.extend(chunk)


    def process_image(self):
        now = time.time()
        if self.last_frame_time is not None:
            delta = now - self.last_frame_time
            fps = 1.0 / delta if delta > 0 else 0.0
            self.node.get_logger().info(f" [{self.cf_id}] Δt={delta:.3f}s → FPS={fps:.2f}")
        self.last_frame_time = now

        try:
            np_data = np.frombuffer(self.buffer, np.uint8)
            decoded = cv2.imdecode(np_data, cv2.IMREAD_UNCHANGED)
            if decoded is None:
                self.node.get_logger().warn(f"[{self.cf_id}] Failed to decode image.")
                return

            msg = self.bridge.cv2_to_imgmsg(decoded, encoding='mono8' if len(decoded.shape) == 2 else 'bgr8')
            ros_now = self.node.get_clock().now().to_msg()
            msg.header.stamp = ros_now
            msg.header.frame_id = f"{self.cf_id}/camera"
            self.image_pub.publish(msg)

            if self.camera_info_msg:
                self.camera_info_msg.header.stamp = ros_now
                self.caminfo_pub.publish(self.camera_info_msg)

        except Exception as e:
            self.node.get_logger().error(f"[{self.cf_id}] Decode error: {e}")

        self.receiving = False
        self.expected_size = None


class UDPImageStreamerNode(Node):
    def __init__(self):
        super().__init__("udp_image_streamer")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", UDP_PORT))
        self.sock.setblocking(False)

        self.bridge = CvBridge()
        self.streams = {}

        self.declare_parameter("aideck_config_path", "")
        config_path = self.get_parameter("aideck_config_path").get_parameter_value().string_value
        self.camera_info = None
        self.ip_to_cf = {}

        if config_path:
            import yaml
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
                self.camera_info = config.get("camera_info", None)
                cameras = config.get("cameras", {})
                for cf_id, info in cameras.items():
                    ip = info["deck_ip"]
                    self.ip_to_cf[ip] = cf_id

        self.magic = MagicSender(self, self.sock, self.ip_to_cf)

        self.get_logger().info(f" UDP server listening on port {UDP_PORT}")
        self.timer = self.create_timer(0.001, self.poll_udp)

        self.magic_timer = self.create_timer(1.0, self.magic.check_and_send_magic)


    def check_and_send_magic(self):
        self.magic.check_and_send_magic()


    def poll_udp(self):
        try:
            data, addr = self.sock.recvfrom(2048)
        except BlockingIOError:
            return

        if addr not in self.streams:
            if len(self.streams) >= 3:
                self.get_logger().warn(f" Ignoring {addr} — max 3 streams active")
                return
            cf_id = self.ip_to_cf.get(addr[0], None)
            self.streams[addr] = PerAddressStream(self, addr, self.bridge, cf_id, self.camera_info)

        stream = self.streams[addr]
        self.magic.update_last_seen(addr[0])

        if len(data) >= CPX_HEADER_SIZE + 1 and data[CPX_HEADER_SIZE] == IMG_HEADER_MAGIC:
            payload = data[CPX_HEADER_SIZE:]
            if len(payload) < IMG_HEADER_SIZE:
                self.get_logger().warn(f"[{stream.cf_id}] Incomplete image header")
                return

            _, width, height, depth, fmt, size = struct.unpack('<BHHBBI', payload[:IMG_HEADER_SIZE])
            stream.update_header(width, height, depth, fmt, size, payload[IMG_HEADER_SIZE:])
            
        elif stream.receiving:
            stream.append_data(data[CPX_HEADER_SIZE:])
            if stream.expected_size is not None and len(stream.buffer) >= stream.expected_size:
                stream.process_image()


def main(args=None):
    rclpy.init(args=args)
    node = UDPImageStreamerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()