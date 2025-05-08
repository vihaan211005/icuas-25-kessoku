#!/usr/bin/env python3
import socket, os, struct
import numpy as np
import yaml
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

import threading
import time

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class AIDeckClient:
    """Client class for a single AI deck"""
    def __init__(self, node, cf_id, deck_ip, deck_port, camera_info_config):
        self.node = node
        self.cf_id = cf_id
        self.image_topic = f"/{cf_id}/image"
        self.camera_info_topic = f"/{cf_id}/camera_info"

        self.image_publisher = node.create_publisher(Image, self.image_topic, 10)
        self.info_publisher = node.create_publisher(CameraInfo, self.camera_info_topic, 10)

        self.deck_ip = deck_ip
        self.deck_port = deck_port

        self.stop_event = threading.Event()
        self.socket_lock = threading.Lock() 

        # Start monitor thread to check if there has been some time since last image was received
        self.last_image_time = time.time()
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()

        self.image_msg = Image()
        self.camera_info_msg = self._construct_camera_info(camera_info_config)

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while True:
            try:
                self.client_socket.connect((deck_ip, deck_port))
                self.node.get_logger().info(f"[{cf_id}] Connected to socket on {deck_ip}:{deck_port}")
                break  # Exit loop on successful connection
            except OSError as e:
                self.node.get_logger().warn(f"[{cf_id}] Retry: failed to connect to {deck_ip}:{deck_port}. Error: {e}")
                time.sleep(1.0)

        self.image = None
        self.thread = None 
        self.thread = threading.Thread(target=self.stream_loop, daemon=True)
        self.thread.start()


    def _construct_camera_info(self, config):
        camera_info = CameraInfo()
        camera_info.header.frame_id = f"{self.cf_id}/camera"
        camera_info.width = int(config['image_width'])
        camera_info.height = int(config['image_height'])
        camera_info.distortion_model = config['distortion_model']
        camera_info.d = config['distortion_coefficients']['data']
        camera_info.k = config['camera_matrix']['data']
        camera_info.r = config['rectification_matrix']['data']
        camera_info.p = config['projection_matrix']['data']
        return camera_info


    def _rx_bytes(self, size):
        data = bytearray()
        while len(data) < size and not self.stop_event.is_set():
            try:
                with self.socket_lock:
                    self.client_socket.settimeout(1.0)  # avoid blocking forever
                    chunk = self.client_socket.recv(size - len(data))
                    if not chunk:
                        return None  # Trigger stream_loop to exit
                    data.extend(chunk)
            except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
                self.node.get_logger().warn(f"[{self.cf_id}] Socket read failed: {e}")
                return None
        return data


    def stream_loop(self):
        def resync_to_header():
            """
            Try to resync by scanning for the magic byte (0xBC) and validating the header.
            Instead of reading selected bytes, we read byte by byte, ensuring that if sync 
            is lost, we can again sync with the server by finding the next magic byte.
            """
            while rclpy.ok() and not self.stop_event.is_set():
                try:
                    magic_byte = self._rx_bytes(1)
                    if magic_byte is None:
                        return None, None, None, None, None  # Socket error or stop requested

                    magic_candidate = magic_byte[0]
                    # self.node.get_logger().info(f"[{self.cf_id}] Byte: 0x{magic_candidate:02X}")
                    if magic_candidate == 0xBC:
                        header_rest = self._rx_bytes(10)
                        if header_rest is None:
                            return None, None, None, None, None

                        full_header = bytes([magic_candidate]) + header_rest
                        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', full_header)

                        # Validate the header data!
                        if width == 324 and height == 244 and depth == 1 and format in [0, 1]:
                            # self.node.get_logger().info(f"[{self.cf_id}] Resynced successfully")
                            return width, height, depth, format, size
                except Exception:
                    continue
            return None, None, None, None, None

        while rclpy.ok() and not self.stop_event.is_set():
            try:
                # Ignore packetInfoRaw entirely, just search for magic byte 0xBC
                width, height, depth, format, size = resync_to_header()
                if None in (width, height, depth, format, size):
                    break

                imgStream = bytearray()
                while len(imgStream) < size and not self.stop_event.is_set():
                    packetInfoRaw = self._rx_bytes(4)
                    if packetInfoRaw is None:
                        break  # socket issue or stop event

                    [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)

                    chunk = self._rx_bytes(length - 2)
                    if chunk is None:
                        break
                    imgStream.extend(chunk)

                if self.stop_event.is_set() or len(imgStream) < size:
                    break

                if format == 0:
                    raw_img = np.frombuffer(imgStream, dtype=np.uint8)
                    raw_img.shape = (height, width)
                else:   # for jpeg
                    nparr = np.frombuffer(imgStream, np.uint8)
                    raw_img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
                    if raw_img is None:
                        self.node.get_logger().warn(f"[{self.cf_id}] Failed to decode JPEG image")
                        continue
                    height, width = raw_img.shape

                # Publish the image
                now = self.node.get_clock().now().to_msg()
                self.image_msg = Image()
                self.image_msg.header.stamp = now
                self.image_msg.header.frame_id = f"{self.cf_id}/camera"
                self.camera_info_msg.header.stamp = now

                self.image_msg.height = height
                self.image_msg.width = width
                self.image_msg.encoding = 'mono8'
                self.image_msg.is_bigendian = 0
                self.image_msg.step = self.image_msg.width
                self.image_msg.data = raw_img.tobytes()

                self.image_publisher.publish(self.image_msg)
                self.info_publisher.publish(self.camera_info_msg)

                self.last_image_time = time.time()

            except Exception as e:
                self.node.get_logger().error(f"[{self.cf_id}] Streaming error during image receive: {e}")
                break

        self.node.get_logger().info(f"[{self.cf_id}] stream_loop exited.")


    def monitor_loop(self):
        """Keeps checking if images are being consistently received."""
        while rclpy.ok():
            time.sleep(1.0)
            if time.time() - self.last_image_time > 2:
                self.node.get_logger().warn(f"[{self.cf_id}] No image received in the last 2 seconds. Attempting to reconnect...")

                # Stop old stream thread
                self.stop_event.set()
                if getattr(self, "thread", None) is not None:
                    if self.thread.is_alive():
                        self.node.get_logger().info(f"[{self.cf_id}] Waiting for old stream thread to finish...")
                        self.thread.join(timeout=2.0)
                        if self.thread.is_alive():
                            self.node.get_logger().warn(f"[{self.cf_id}] WARNING: old stream thread is still alive after join().")
                else:
                    self.node.get_logger().warn(f"[{self.cf_id}] Cannot reconnect — stream thread was never started.")
                    continue  # Skip reconnect and retry on next cycle

                # Begin reconnection attempts
                while rclpy.ok():
                    try:
                        self.node.get_logger().info(f"[{self.cf_id}] Closing old socket...")
                        with self.socket_lock:
                            self.client_socket.close()

                        self.node.get_logger().info(f"[{self.cf_id}] Creating new socket...")
                        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.client_socket.settimeout(10)

                        self.node.get_logger().info(f"[{self.cf_id}] Attempting to connect to {self.deck_ip}:{self.deck_port}...")
                        self.client_socket.connect((self.deck_ip, self.deck_port))
                        self.node.get_logger().info(f"[{self.cf_id}] Successfully reconnected.")
                        break

                    except Exception as e:
                        self.node.get_logger().warn(f"[{self.cf_id}] Reconnect failed: {e}")
                        time.sleep(1.0)

                # Restart stream thread
                self.node.get_logger().info(f"[{self.cf_id}] Starting new stream thread...")
                self.stop_event.clear()
                self.thread = threading.Thread(target=self.stream_loop, daemon=True)
                self.thread.start()

                # Reset timer
                self.last_image_time = time.time()


class ImageStreamerNode(Node):
    def __init__(self):
        super().__init__("image_streamer")

        self.declare_parameter("aideck_config_path", "")
        config_path = self.get_parameter("aideck_config_path").get_parameter_value().string_value
        if not config_path:
            self.get_logger().error("No config path provided in 'aideck_config_path' parameter.")
            raise RuntimeError("Missing 'aideck_config_path' parameter")
        with open(config_path, 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        camera_info = config["camera_info"]
        cameras = config["cameras"]

        self.streamers = []
        for cf_id, cam in cameras.items():
            client = AIDeckClient(self, cf_id, cam["deck_ip"], cam["deck_port"], camera_info)
            self.streamers.append(client)
            self.publish_static_camera_transform(self, cf_id)
          
        self.get_logger().info(f"Started streaming for: {', '.join(cameras.keys())}")


    def publish_static_camera_transform(self, node, cf_id):
        """
        Publishes a static transform from cf_id -> cf_id/camera with camera convention:
        - Z forward
        - X right
        - Y down
        """
        broadcaster = StaticTransformBroadcaster(node)

        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = cf_id  # Parent is drone body frame
        t.child_frame_id = f"{cf_id}/camera"

        # Position of the camera in drone frame (e.g., 2cm forward and 1cm down)
        t.transform.translation.x = 0.02
        t.transform.translation.y = 0.0
        t.transform.translation.z = -0.01

        # Orientation: camera Z forward, X right, Y down (camera optical frame)
        quat = tf_transformations.quaternion_from_euler(-np.pi/2, 0, -np.pi/2)  # Rotate to camera frame convention
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImageStreamerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()