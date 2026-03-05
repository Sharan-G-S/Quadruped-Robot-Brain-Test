"""
ekf_node.py — Extended Kalman Filter for Sensor Fusion
========================================================
Fuses IMU + odometry (+ optional GPS) to produce a filtered state estimate.
Publishes nav_msgs/Odometry with the robot's estimated pose and velocity.

State vector [9]: [x, y, z, vx, vy, vz, roll, pitch, yaw]
"""

import sys
import os
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Point, Quaternion, Twist, Vector3,
    PoseWithCovariance, TwistWithCovariance,
)
from std_msgs.msg import Header
import numpy as np


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert Euler angles (radians) to quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class EKFNode(Node):
    """
    Extended Kalman Filter node for robot pose estimation.

    Prediction: Uses IMU accelerometer + gyroscope
    Update:     Uses IMU orientation (gravity-based roll/pitch)
    """

    def __init__(self):
        super().__init__("ekf_node")

        # Parameters
        self.declare_parameter("process_noise_pos", 0.01)
        self.declare_parameter("process_noise_vel", 0.1)
        self.declare_parameter("process_noise_ori", 0.01)
        self.declare_parameter("measurement_noise_accel", 0.5)
        self.declare_parameter("measurement_noise_gyro", 0.05)
        self.declare_parameter("publish_rate", 50.0)

        # Get params
        pn_pos = self.get_parameter("process_noise_pos").value
        pn_vel = self.get_parameter("process_noise_vel").value
        pn_ori = self.get_parameter("process_noise_ori").value
        mn_accel = self.get_parameter("measurement_noise_accel").value
        mn_gyro = self.get_parameter("measurement_noise_gyro").value
        rate = self.get_parameter("publish_rate").value

        # ── EKF State ──────────────────────────────────────────
        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.n = 9
        self.x = np.zeros(self.n)            # State vector
        self.P = np.eye(self.n) * 1.0        # State covariance

        # Process noise
        self.Q = np.diag([
            pn_pos, pn_pos, pn_pos,          # position
            pn_vel, pn_vel, pn_vel,           # velocity
            pn_ori, pn_ori, pn_ori,           # orientation
        ])

        # Measurement noise for IMU
        self.R_imu = np.diag([
            mn_accel, mn_accel, mn_accel,     # accelerometer
            mn_gyro, mn_gyro, mn_gyro,        # gyroscope
        ])

        self._last_time = time.time()

        # ── ROS2 Interface ─────────────────────────────────────
        self.imu_sub = self.create_subscription(
            Imu, "/robot_dog/imu/data", self.imu_callback, 10
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/robot_dog/odom", 10
        )
        self.pub_timer = self.create_timer(1.0 / rate, self.publish_odom)

        self.get_logger().info("EKF node started (9-state)")

    # ── IMU Callback ────────────────────────────────────────────

    def imu_callback(self, msg: Imu):
        """Process IMU measurement: predict + update step."""
        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        if dt <= 0 or dt > 1.0:
            dt = 0.02  # Fallback

        # Extract IMU values
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # ── PREDICT ─────────────────────────────────────────
        self._predict(dt, ax, ay, az, gx, gy, gz)

        # ── UPDATE ──────────────────────────────────────────
        self._update_imu(ax, ay, az, gx, gy, gz)

    def _predict(self, dt: float, ax: float, ay: float, az: float,
                 gx: float, gy: float, gz: float):
        """
        EKF Prediction step using IMU data.
        Integrates acceleration to update velocity and position.
        Integrates gyroscope to update orientation.
        """
        roll, pitch, yaw = self.x[6], self.x[7], self.x[8]

        # Rotate body-frame acceleration to world frame
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        # Rotation matrix (body → world)
        R = np.array([
            [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
            [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
            [-sp,    cp*sr,             cp*cr],
        ])

        accel_body = np.array([ax, ay, az - 9.81])  # Remove gravity
        accel_world = R @ accel_body

        # State transition
        x_pred = self.x.copy()

        # Position: x += v*dt + 0.5*a*dt²
        x_pred[0] += self.x[3] * dt + 0.5 * accel_world[0] * dt**2
        x_pred[1] += self.x[4] * dt + 0.5 * accel_world[1] * dt**2
        x_pred[2] += self.x[5] * dt + 0.5 * accel_world[2] * dt**2

        # Velocity: v += a*dt
        x_pred[3] += accel_world[0] * dt
        x_pred[4] += accel_world[1] * dt
        x_pred[5] += accel_world[2] * dt

        # Orientation: integrate gyroscope
        x_pred[6] += gx * dt
        x_pred[7] += gy * dt
        x_pred[8] += gz * dt

        # Normalize angles to [-pi, pi]
        for i in [6, 7, 8]:
            x_pred[i] = math.atan2(math.sin(x_pred[i]), math.cos(x_pred[i]))

        self.x = x_pred

        # Jacobian of state transition (F ≈ I + df/dx * dt)
        F = np.eye(self.n)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q * dt

    def _update_imu(self, ax: float, ay: float, az: float,
                    gx: float, gy: float, gz: float):
        """
        EKF Update step using IMU accelerometer for roll/pitch correction.
        Accelerometer provides an absolute reference via gravity.
        """
        # Measurement: roll and pitch from accelerometer
        accel_roll = math.atan2(ay, math.sqrt(ax**2 + az**2))
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        # Measurement vector (what we observe)
        z = np.array([accel_roll, accel_pitch])

        # Predicted measurement (from state)
        z_pred = np.array([self.x[6], self.x[7]])

        # Innovation (measurement residual)
        y = z - z_pred

        # Measurement matrix (H) — maps state to measurement space
        H = np.zeros((2, self.n))
        H[0, 6] = 1.0  # roll
        H[1, 7] = 1.0  # pitch

        # Measurement noise for roll/pitch
        R = np.diag([self.R_imu[0, 0], self.R_imu[1, 1]])

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return  # Skip update if singular

        # State update
        self.x = self.x + K @ y

        # Covariance update (Joseph form for stability)
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    # ── Publish Odometry ────────────────────────────────────────

    def publish_odom(self):
        """Publish the filtered state as nav_msgs/Odometry."""
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Position
        msg.pose.pose.position = Point(
            x=self.x[0], y=self.x[1], z=self.x[2]
        )

        # Orientation (quaternion from Euler)
        msg.pose.pose.orientation = euler_to_quaternion(
            self.x[6], self.x[7], self.x[8]
        )

        # Velocity
        msg.twist.twist.linear = Vector3(
            x=self.x[3], y=self.x[4], z=self.x[5]
        )

        # Covariance (6×6 subset of P for pose)
        pose_cov = np.zeros(36)
        twist_cov = np.zeros(36)
        # Map position and orientation covariance
        for i in range(3):
            pose_cov[i * 7] = self.P[i, i]        # Position diagonal
            pose_cov[(i+3) * 7] = self.P[i+6, i+6] # Orientation diagonal
            twist_cov[i * 7] = self.P[i+3, i+3]    # Velocity diagonal
        msg.pose.covariance = pose_cov.tolist()
        msg.twist.covariance = twist_cov.tolist()

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
