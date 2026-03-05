"""
sensor_publisher.py — ROS2 Sensor Publisher Node
==================================================
Publishes IMU, distance, and camera data to ROS2 topics.
"""

import sys
import os
import time

# Add project root to path so we can import our hardware modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range, Image
from std_msgs.msg import Header
import numpy as np

from hardware.imu_sensor import IMUSensor
from hardware.distance_sensor import DistanceSensor
from hardware.camera_module import CameraModule


class SensorPublisher(Node):
    """ROS2 node that reads sensors and publishes to topics."""

    def __init__(self):
        super().__init__("sensor_publisher")

        # Declare parameters
        self.declare_parameter("imu_type", "simulation")
        self.declare_parameter("distance_type", "simulation")
        self.declare_parameter("camera_device", 0)
        self.declare_parameter("publish_rate", 50.0)

        # Initialize sensors
        imu_type = self.get_parameter("imu_type").value
        dist_type = self.get_parameter("distance_type").value
        cam_device = self.get_parameter("camera_device").value
        rate = self.get_parameter("publish_rate").value

        self.imu = IMUSensor({"type": imu_type})
        self.distance = DistanceSensor({"type": dist_type})
        self.camera = CameraModule({"device_id": cam_device})

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/robot_dog/imu/data", 10)
        self.dist_pub = self.create_publisher(Range, "/robot_dog/distance", 10)
        self.cam_pub = self.create_publisher(Image, "/robot_dog/camera/image_raw", 5)

        # Timers
        self.imu_timer = self.create_timer(1.0 / rate, self.publish_imu)
        self.dist_timer = self.create_timer(0.05, self.publish_distance)  # 20Hz
        self.cam_timer = self.create_timer(0.1, self.publish_camera)     # 10Hz

        self.get_logger().info("✅ SensorPublisher started")

    def publish_imu(self):
        data = self.imu.read()
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Linear acceleration
        msg.linear_acceleration.x = data["accel_x"]
        msg.linear_acceleration.y = data["accel_y"]
        msg.linear_acceleration.z = data["accel_z"]

        # Angular velocity
        msg.angular_velocity.x = data["gyro_x"]
        msg.angular_velocity.y = data["gyro_y"]
        msg.angular_velocity.z = data["gyro_z"]

        self.imu_pub.publish(msg)

    def publish_distance(self):
        dist = self.distance.read()
        msg = Range()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "distance_sensor"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees
        msg.min_range = self.distance.min_dist / 100.0  # m
        msg.max_range = self.distance.max_dist / 100.0  # m
        msg.range = dist / 100.0  # cm → m
        self.dist_pub.publish(msg)

    def publish_camera(self):
        frame = self.camera.capture_frame()
        if frame is None:
            return
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.cam_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.camera.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
