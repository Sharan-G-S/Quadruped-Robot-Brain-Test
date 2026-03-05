"""
motor_controller_node.py — ROS2 Motor Controller Subscriber
=============================================================
Subscribes to /cmd_vel (Twist) and translates velocity commands
into gait patterns and servo angles.
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

from locomotion.kinematics import InverseKinematics
from locomotion.gait_controller import GaitController, GaitType
from locomotion.body_controller import BodyController
from hardware.servo_driver import ServoDriver


class MotorControllerNode(Node):
    """ROS2 node that converts velocity commands to servo angles."""

    def __init__(self):
        super().__init__("motor_controller")

        # Parameters
        self.declare_parameter("control_rate", 50.0)

        rate = self.get_parameter("control_rate").value

        # Load configs (defaults for simulation)
        gait_cfg = {
            "step_height": 30.0, "step_length": 60.0,
            "cycle_time": 0.8, "trot_speed": 0.5,
        }
        body_cfg = {
            "length": 200.0, "width": 110.0, "default_height": 150.0,
        }
        leg_cfg = {
            "hip_length": 50.0, "upper_length": 107.0, "lower_length": 130.0,
        }
        servo_cfg = {
            "channels": {"FR": [0,1,2], "FL": [3,4,5],
                         "RR": [6,7,8], "RL": [9,10,11]},
            "offsets": [0]*12,
        }

        # Initialize locomotion stack
        self.body = BodyController(gait_cfg, body_cfg, leg_cfg)
        self.servos = ServoDriver(servo_cfg)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/robot_dog/cmd_vel", self.cmd_vel_callback, 10
        )
        self.action_sub = self.create_subscription(
            String, "/robot_dog/brain/command", self.action_callback, 10
        )

        # Publisher for current state
        self.state_pub = self.create_publisher(
            String, "/robot_dog/state", 10
        )

        # Control loop timer
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self._last_cmd_vel = Twist()
        self.get_logger().info("MotorController started")

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands (compatible with teleop_twist)."""
        self._last_cmd_vel = msg

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Map velocities to gaits
        if abs(linear_x) < 0.05 and abs(angular_z) < 0.05:
            self.body.stand()
        elif abs(angular_z) > 0.3:
            if angular_z > 0:
                self.body.turn_left(min(1.0, abs(angular_z)))
            else:
                self.body.turn_right(min(1.0, abs(angular_z)))
        elif linear_x > 0.05:
            speed = min(1.0, abs(linear_x))
            if speed > 0.6:
                self.body.trot(speed)
            else:
                self.body.walk(speed)
        elif linear_x < -0.05:
            # Backward walking — use walk with reduced speed
            self.body.walk(min(0.3, abs(linear_x)))

    def action_callback(self, msg: String):
        """Handle high-level action commands from the brain node."""
        try:
            action = json.loads(msg.data)
            self.body.execute_action(action)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid action JSON: {msg.data}")

    def control_loop(self):
        """Main control loop — compute and send servo angles."""
        # Get current servo angles from body controller
        angles = self.body.update()

        # Send to servo driver
        self.servos.set_all_legs(angles)

        # Publish state
        state_msg = String()
        state_msg.data = json.dumps({
            "gait": self.body.gait.current_gait.value,
            "angles": angles,
        })
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.servos.relax()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
