#!/usr/bin/env python3
"""
main.py — QuadBot-AI System Entry Point
=========================================
Initializes all subsystems and runs the main control loop.

Usage:
    python main.py                 # Default: simulation mode
    python main.py --mode serial   # Serial (Arduino/ESP32)
    python main.py --mode ros2     # ROS2 mode
    python main.py --mode simulation --no-dashboard
"""

import argparse
import logging
import signal
import sys
import time
import threading
import os
from typing import Optional

import yaml

# ── Setup Logging ───────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s │ %(name)-20s │ %(levelname)-5s │ %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("QuadBot")


# ── Load Config ─────────────────────────────────────────────────

def load_config(path: str = "config/robot_config.yaml") -> dict:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    full_path = os.path.join(script_dir, path)
    try:
        with open(full_path, "r") as f:
            config = yaml.safe_load(f)
        logger.info(f"Config loaded: {full_path}")
        return config
    except Exception as e:
        logger.warning(f"Config load failed ({e}), using defaults")
        return {}


# ── Robot Class ─────────────────────────────────────────────────

class QuadBotAI:
    """
    Main robot orchestrator — connects all subsystems together.

    Pipeline per loop iteration:
      1. Read sensors (IMU, distance, camera)
      2. Safety check
      3. Vision analysis (periodic)
      4. Decision making
      5. Locomotion update (gait → IK → servos)
      6. Communication (serial/ROS2)
      7. Telemetry broadcast
    """

    def __init__(self, config: dict, mode: str = "simulation"):
        self.config = config
        self.mode = mode
        self._running = False
        self._loop_hz = config.get("system", {}).get("control_loop_hz", 50)

        logger.info(f"QuadBot-AI initializing (mode={mode})")

        # ── Hardware ────────────────────────────────────────
        from hardware.servo_driver import ServoDriver
        from hardware.imu_sensor import IMUSensor
        from hardware.distance_sensor import DistanceSensor
        from hardware.camera_module import CameraModule

        servo_cfg = config.get("servos", {})
        imu_cfg = config.get("imu", {})
        dist_cfg = config.get("distance_sensor", {})
        cam_cfg = config.get("camera", {})

        # Force simulation if mode is simulation
        if mode == "simulation":
            imu_cfg["type"] = "simulation"
            dist_cfg["type"] = "simulation"

        self.servos = ServoDriver(servo_cfg)
        self.imu = IMUSensor(imu_cfg)
        self.distance = DistanceSensor(dist_cfg)
        self.camera = CameraModule(cam_cfg)

        # ── Locomotion ──────────────────────────────────────
        from locomotion.body_controller import BodyController

        self.body = BodyController(
            config.get("gait", {}),
            config.get("body", {}),
            config.get("leg", {}),
        )

        # ── Brain ───────────────────────────────────────────
        from brain.llm_client import LLMClient
        from brain.vision_processor import VisionProcessor
        from brain.decision_engine import DecisionEngine
        from brain.command_parser import CommandParser
        from brain.state_machine import StateMachine, RobotState
        from brain.safety_system import SafetySystem

        llm_cfg = config.get("llm", {})
        safety_cfg = config.get("safety", {})

        self.llm = LLMClient(llm_cfg)
        self.vision = VisionProcessor(self.llm, llm_cfg)
        self.decision = DecisionEngine(self.llm)
        self.command_parser = CommandParser(self.llm)
        self.state_machine = StateMachine()
        self.safety = SafetySystem(safety_cfg)

        # Wire safety to state machine
        self.safety.set_emergency_callback(
            lambda reason: self.state_machine.emergency_stop(reason)
        )

        # ── Communication ───────────────────────────────────
        self.serial_bridge = None
        if mode == "serial":
            from communication.serial_bridge import SerialBridge
            self.serial_bridge = SerialBridge(config.get("serial", {}))

        # ── Boot ────────────────────────────────────────────
        self.state_machine.transition(RobotState.STANDING, "boot")
        self.body.stand()
        self.servos.center_all()

        logger.info("QuadBot-AI fully initialized")

    # ── Main Control Loop ───────────────────────────────────────

    def run(self):
        """Start the main control loop."""
        self._running = True
        loop_period = 1.0 / self._loop_hz
        logger.info(f"Control loop starting at {self._loop_hz} Hz")

        iteration = 0
        while self._running:
            start = time.time()
            iteration += 1

            try:
                # 1. Read sensors
                imu_data = self.imu.read()
                dist_val = self.distance.read()
                dist_data = self.distance.get_telemetry()

                # 2. Safety check
                is_safe = self.safety.check(
                    imu_data=imu_data,
                    distance_data=dist_data,
                )

                if not is_safe:
                    self.body.stand()
                    if self.serial_bridge:
                        self.serial_bridge.send_emergency_stop()
                else:
                    # 3. Vision analysis (periodic, not every loop)
                    if self.vision.should_analyze():
                        frame = self.camera.capture_frame()
                        if frame is not None:
                            b64 = self.camera.frame_to_base64(frame)
                            if b64:
                                ctx = f"State: {self.state_machine.state.value}"
                                self.vision.analyze_frame(b64, ctx)

                    # 4. Decision making (every ~10 iterations)
                    if iteration % 10 == 0:
                        decision = self.decision.decide(
                            vision_result=self.vision.last_result,
                            sensor_data=dist_data,
                            current_state=self.state_machine.state.value,
                        )
                        action = decision.get("action", "stop")
                        self.body.execute_action(decision)
                        self.state_machine.apply_action(
                            action, decision.get("reasoning", ""))

                    # 5. IMU stabilization
                    self.body.compensate_tilt(
                        imu_data.get("roll", 0),
                        imu_data.get("pitch", 0),
                    )

                    # 6. Locomotion update → servo angles
                    angles = self.body.update()
                    self.servos.set_all_legs(angles)

                    # 7. Serial communication
                    if self.serial_bridge:
                        self.serial_bridge.send_all_legs(
                            self.config.get("servos", {}).get("channels", {}),
                            angles,
                        )
                        self.serial_bridge.send_heartbeat()

            except Exception as e:
                logger.error(f"Control loop error: {e}")

            # Maintain loop rate
            elapsed = time.time() - start
            sleep_time = loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        """Stop the robot gracefully."""
        logger.info("Stopping QuadBot-AI...")
        self._running = False
        self.body.stand()
        time.sleep(0.3)
        self.servos.relax()
        self.camera.release()
        self.distance.cleanup()
        if self.serial_bridge:
            self.serial_bridge.stop()
        logger.info("QuadBot-AI stopped")

    # ── External Interface ──────────────────────────────────────

    def process_command(self, text: str):
        """Process a natural language command."""
        parsed = self.command_parser.parse(text)
        if parsed.get("confidence", 0) > 0.3:
            self.body.execute_action(parsed)
            action = parsed.get("action", "stop")
            self.state_machine.apply_action(action, f"user: {text}")
            logger.info(f"Command: '{text}' -> {action}")

    def execute_action(self, action: dict):
        """Execute a direct action dict."""
        self.body.execute_action(action)
        a = action.get("action", "stop")
        self.state_machine.apply_action(a, "direct action")

    def emergency_stop(self):
        """Trigger emergency stop."""
        from brain.state_machine import RobotState
        self.state_machine.transition(RobotState.EMERGENCY_STOP, "user e-stop")
        self.body.stand()
        self.servos.relax()
        if self.serial_bridge:
            self.serial_bridge.send_emergency_stop()

    def get_telemetry(self) -> dict:
        """Get full system telemetry for dashboard."""
        return {
            "mode": self.mode,
            "state_machine": self.state_machine.get_telemetry(),
            "body": self.body.get_telemetry(),
            "imu": self.imu.get_telemetry(),
            "distance": self.distance.get_telemetry(),
            "camera": self.camera.get_telemetry(),
            "servos": self.servos.get_telemetry(),
            "safety": self.safety.get_telemetry(),
            "vision": self.vision.get_telemetry(),
            "decision": self.decision.get_telemetry(),
            "serial": self.serial_bridge.get_telemetry() if self.serial_bridge else None,
        }


# ── Main ────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="QuadBot-AI — Intelligent Robot Dog")
    parser.add_argument("--mode", choices=["simulation", "serial", "ros2"],
                        default="simulation", help="Operation mode")
    parser.add_argument("--config", default="config/robot_config.yaml",
                        help="Path to config file")
    parser.add_argument("--no-dashboard", action="store_true",
                        help="Disable web dashboard")
    args = parser.parse_args()

    # ROS2 mode: use ros2 launch instead
    if args.mode == "ros2":
        logger.info("ROS2 mode — use: ros2 launch robot_dog robot_dog_launch.py")
        logger.info("Or run individual nodes with: ros2 run robot_dog <node_name>")
        sys.exit(0)

    config = load_config(args.config)
    robot = QuadBotAI(config, mode=args.mode)

    # Graceful shutdown
    def signal_handler(sig, frame):
        robot.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Start dashboard in background thread
    if not args.no_dashboard:
        from dashboard.server import set_robot, run_dashboard
        set_robot(robot)
        dash_cfg = config.get("dashboard", {})
        dash_thread = threading.Thread(
            target=run_dashboard,
            kwargs={
                "host": dash_cfg.get("host", "0.0.0.0"),
                "port": dash_cfg.get("port", 8080),
            },
            daemon=True,
        )
        dash_thread.start()
        logger.info(f"Dashboard: http://localhost:{dash_cfg.get('port', 8080)}")

    # Run main control loop
    robot.run()


if __name__ == "__main__":
    main()
