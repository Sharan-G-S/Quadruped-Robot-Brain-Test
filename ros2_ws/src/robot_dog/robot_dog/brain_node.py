"""
brain_node.py — ROS2 LLM Brain Node
======================================
Wraps the LLM vision brain as a ROS2 node.
Subscribes to camera/sensor topics, runs vision analysis,
publishes movement commands.
"""

import sys
import os
import json
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import yaml

from brain.llm_client import LLMClient
from brain.vision_processor import VisionProcessor
from brain.decision_engine import DecisionEngine
from brain.command_parser import CommandParser
from brain.state_machine import StateMachine, RobotState
from brain.safety_system import SafetySystem


class BrainNode(Node):
    """ROS2 node that integrates LLM vision, decision-making, and safety."""

    def __init__(self):
        super().__init__("brain_node")

        # Parameters
        self.declare_parameter("config_path",
            os.path.join(os.path.dirname(__file__),
                         "..", "..", "..", "..", "config", "robot_config.yaml"))
        self.declare_parameter("vision_interval", 2.0)

        config_path = self.get_parameter("config_path").value
        self._vision_interval = self.get_parameter("vision_interval").value

        # Load config
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warning(f"Config load failed: {e}, using defaults")
            config = {}

        # Initialize brain components
        llm_config = config.get("llm", {})
        safety_config = config.get("safety", {})

        self.llm = LLMClient(llm_config)
        self.vision = VisionProcessor(self.llm, llm_config)
        self.decision = DecisionEngine(self.llm)
        self.command_parser = CommandParser(self.llm)
        self.state_machine = StateMachine()
        self.safety = SafetySystem(safety_config)

        # Wire safety emergency to state machine
        self.safety.set_emergency_callback(
            lambda reason: self.state_machine.emergency_stop(reason)
        )

        # Latest data
        self._latest_imu = None
        self._latest_distance = None
        self._latest_odom = None
        self._latest_frame_b64 = None
        self._last_vision_time = 0

        # ── Subscribers ─────────────────────────────────────
        self.imu_sub = self.create_subscription(
            Imu, "/robot_dog/imu/data", self.imu_cb, 10)
        self.dist_sub = self.create_subscription(
            Range, "/robot_dog/distance", self.dist_cb, 10)
        self.cam_sub = self.create_subscription(
            Image, "/robot_dog/camera/image_raw", self.cam_cb, 5)
        self.odom_sub = self.create_subscription(
            Odometry, "/robot_dog/odom", self.odom_cb, 10)
        self.user_cmd_sub = self.create_subscription(
            String, "/robot_dog/user_command", self.user_cmd_cb, 10)

        # ── Publishers ──────────────────────────────────────
        self.action_pub = self.create_publisher(
            String, "/robot_dog/brain/command", 10)
        self.state_pub = self.create_publisher(
            String, "/robot_dog/brain/state", 10)

        # ── Main thinking loop (2 Hz) ──────────────────────
        self.think_timer = self.create_timer(0.5, self.think_loop)

        # Start in standing state
        self.state_machine.transition(RobotState.STANDING, "boot")

        self.get_logger().info("BrainNode started -- LLM-powered intelligence active")

    # ── Callbacks ───────────────────────────────────────────────

    def imu_cb(self, msg: Imu):
        self._latest_imu = {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,  # Computed by EKF
            "accel_x": msg.linear_acceleration.x,
            "accel_y": msg.linear_acceleration.y,
            "accel_z": msg.linear_acceleration.z,
            "gyro_x": msg.angular_velocity.x,
            "gyro_y": msg.angular_velocity.y,
            "gyro_z": msg.angular_velocity.z,
        }

    def dist_cb(self, msg: Range):
        self._latest_distance = {
            "distance_cm": msg.range * 100,
            "obstacle_detected": msg.range < 0.25,
        }

    def cam_cb(self, msg: Image):
        """Convert ROS Image to base64 for LLM."""
        try:
            import cv2
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            import base64
            self._latest_frame_b64 = base64.b64encode(buf).decode("utf-8")
        except Exception as e:
            self.get_logger().debug(f"Frame encode error: {e}")

    def odom_cb(self, msg: Odometry):
        self._latest_odom = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "vx": msg.twist.twist.linear.x,
            "vy": msg.twist.twist.linear.y,
        }

    def user_cmd_cb(self, msg: String):
        """Handle user text commands."""
        parsed = self.command_parser.parse(msg.data)
        if parsed.get("confidence", 0) > 0.3:
            self._publish_action(parsed)

    # ── Main Thinking Loop ──────────────────────────────────────

    def think_loop(self):
        """
        Main brain loop:
          1. Safety check
          2. Vision analysis (periodic)
          3. Decision making
          4. Action publishing
        """
        # 1. Safety check
        is_safe = self.safety.check(
            imu_data=self._latest_imu,
            distance_data=self._latest_distance,
        )
        if not is_safe:
            self._publish_action({"action": "stop", "speed": 0,
                                  "reasoning": "safety override"})
            self._publish_state()
            return

        # 2. Vision analysis (periodic)
        vision_result = None
        now = time.time()
        if (self._latest_frame_b64 and
                now - self._last_vision_time >= self._vision_interval):
            context = f"State: {self.state_machine.state.value}"
            if self._latest_odom:
                context += f", Pos: ({self._latest_odom['x']:.1f}, {self._latest_odom['y']:.1f})"
            vision_result = self.vision.analyze_frame(
                self._latest_frame_b64, context)
            self._last_vision_time = now

        # 3. Decision
        decision = self.decision.decide(
            vision_result=vision_result,
            sensor_data=self._latest_distance,
            current_state=self.state_machine.state.value,
        )

        # 4. Apply decision
        action = decision.get("action", "stop")
        self.state_machine.apply_action(action, decision.get("reasoning", ""))
        self._publish_action(decision)
        self._publish_state()

    # ── Publishers ──────────────────────────────────────────────

    def _publish_action(self, action: dict):
        msg = String()
        msg.data = json.dumps(action)
        self.action_pub.publish(msg)

    def _publish_state(self):
        msg = String()
        msg.data = json.dumps({
            "state": self.state_machine.state.value,
            "safety": self.safety.get_telemetry(),
            "vision": self.vision.get_telemetry(),
            "decision": self.decision.get_telemetry(),
        })
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
