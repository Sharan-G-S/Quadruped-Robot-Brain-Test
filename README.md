<p align="center">
  <h1 align="center">QuadBot-AI -- Intelligent Robot Dog</h1>
  <p align="center">
    <strong>LLM-Powered Quadruped Robot with Manual + Autonomous Control, Computer Vision, ROS2, and EKF Sensor Fusion</strong>
  </p>
  <p align="center">
    <em>Built by Sharan G S</em>
  </p>
</p>

---

## Architecture

```
+------------------------------------------------------------------+
|                        QuadBot-AI System                          |
+===================== CONTROL LAYER ==========================+
|                                                                  |
|   +-------------+   +--------------+   +---------------------+  |
|   | Mode Manager|   |   Manual     |   |   Autonomous        |  |
|   | (manual /   |-->|  Controller  |   |   Controller        |  |
|   |  auto /     |   | (joystick/  |   | (LLM + Vision)      |  |
|   |  hybrid)    |   |  keyboard)  |   |                     |  |
|   +-------------+   +--------------+   +---------------------+  |
|                                                                  |
+========================= BRAIN ==================================+
|                                                                  |
|   +-------------+   +--------------+   +---------------------+  |
|   |  LLM Vision |   |   Decision   |   |   State Machine     |  |
|   |  (GPT-4o /  |-->|   Engine     |-->|   & Behavior        |  |
|   |   Gemini)   |   |              |   |   Controller        |  |
|   +------+------+   +--------------+   +----------+----------+  |
|          |                                         |             |
|   +------+------+                        +---------+----------+  |
|   |   Camera    |                        |  Safety System     |  |
|   |   Module    |                        |  (Fall Detect,     |  |
|   |  (OpenCV)   |                        |   E-Stop)          |  |
|   +-------------+                        +--------------------+  |
|                                                                  |
+======================== LOCOMOTION ==============================+
|                                                                  |
|   +-------------+   +--------------+   +---------------------+  |
|   |  Inverse    |   |    Gait      |   |   Body Pose         |  |
|   |  Kinematics |<--|  Controller  |<--|   Controller        |  |
|   |  (3-DOF)    |   | (Walk/Trot)  |   |                     |  |
|   +------+------+   +--------------+   +---------------------+  |
|          |                                                       |
+----------+------------- COMMUNICATION --------------------------+
|          |                                                       |
|   +------v------+   +--------------+   +---------------------+  |
|   |   Serial    |   |    ROS2      |   |   EKF Sensor        |  |
|   |   Bridge    |   |   Nodes      |   |   Fusion            |  |
|   | (Arduino)   |   |              |   |                     |  |
|   +------+------+   +------+-------+   +---------------------+  |
|          |                 |                                     |
+----------+-----------------+---- HARDWARE -----------------------+
|          |                 |                                     |
|   +------v------+   +-----v--------+   +---------------------+  |
|   |  12x Servos |   |  IMU Sensor  |   |  Distance Sensor    |  |
|   |  (PCA9685)  |   |  (MPU6050)   |   |  (Ultrasonic)       |  |
|   +-------------+   +--------------+   +---------------------+  |
|                                                                  |
+========================= DASHBOARD =============================+
|   FastAPI + WebSocket | Camera | Telemetry | Joystick | Mode SW  |
+------------------------------------------------------------------+
```

---

## Subsystems

### 1. Control Subsystem (`control/`)

Manages how the robot receives commands -- manually, autonomously, or both.

| File | Purpose |
|------|---------|
| `mode_manager.py` | Switches between `manual`, `autonomous`, and `hybrid` modes |
| `manual_controller.py` | Processes direct user input (dashboard, keyboard, joystick) |
| `autonomous_controller.py` | Wraps LLM/vision pipeline, can be enabled/disabled |

**Control Modes:**

| Mode | Manual Input | LLM/Vision AI | Use Case |
|------|:-----------:|:-------------:|----------|
| `manual` | Active | Disabled | Direct teleoperation, testing |
| `autonomous` | Disabled | Active | Fully AI-driven exploration |
| `hybrid` | Active (priority) | Active (fallback) | Supervised autonomy |

---

### 2. Brain/AI Subsystem (`brain/`)

The intelligence layer -- LLM-powered scene understanding and decision making.

| File | Purpose |
|------|---------|
| `llm_client.py` | OpenAI GPT-4o + Google Gemini API wrapper |
| `vision_processor.py` | Camera frame to LLM scene analysis |
| `decision_engine.py` | Sensor data + vision to robot action mapping |
| `command_parser.py` | Natural language to structured commands |
| `state_machine.py` | Robot behavior states + transition validation |
| `safety_system.py` | Fall detection, tilt limits, emergency stop |

**Robot States:** `IDLE`, `STANDING`, `WALKING`, `TROTTING`, `TURNING`, `EXPLORING`, `OBSTACLE_AVOIDANCE`, `FOLLOWING`, `SITTING`, `LAYING_DOWN`, `MANUAL_CONTROL`, `EMERGENCY_STOP`

---

### 3. Locomotion Subsystem (`locomotion/`)

Generates motion -- from gait patterns through inverse kinematics to servo angles.

| File | Purpose |
|------|---------|
| `kinematics.py` | 3-DOF inverse kinematics solver per leg |
| `gait_controller.py` | Walk, trot, turn, side-step gait patterns with phase timing |
| `body_controller.py` | Body pose, height, lean, and IMU stabilization |

**Pipeline:** `GaitController` (foot positions) -> `InverseKinematics` (angles) -> `ServoDriver` (PWM)

---

### 4. Hardware Subsystem (`hardware/`)

Hardware abstraction layer with automatic simulation fallback.

| File | Purpose |
|------|---------|
| `servo_driver.py` | PCA9685 I2C PWM controller for 12 servos (3-DOF x 4 legs) |
| `imu_sensor.py` | MPU6050/BNO055 IMU with complementary filter |
| `distance_sensor.py` | HC-SR04 ultrasonic / TFMini LiDAR with rolling-average |
| `camera_module.py` | OpenCV capture, base64/JPEG encoding, simulation renderer |

---

### 5. Communication Subsystem (`communication/`)

Serial protocol for SBC-to-microcontroller communication.

| File | Purpose |
|------|---------|
| `serial_protocol.py` | Binary packet encoding/decoding `[0xAA, CMD, LEN, DATA, CHECKSUM]` |
| `serial_bridge.py` | PySerial bridge with background RX thread and heartbeat |

**Protocol Commands:**

| CMD | Name | Data |
|-----|------|------|
| 0x01 | SERVO_WRITE | channel (1B) + angle (2B) |
| 0x02 | SERVO_MULTI | count + [ch+angle] x N |
| 0x03 | SENSOR_REQ | sensor_id (1B) |
| 0x04 | SENSOR_DATA | sensor_id + data |
| 0x05 | HEARTBEAT | timestamp (4B) |
| 0xFF | E_STOP | -- |

---

### 6. Firmware Subsystem (`firmware/`)

| File | Purpose |
|------|---------|
| `arduino_firmware/arduino_firmware.ino` | Arduino/ESP32 real-time servo control and sensor reading |

---

### 7. Dashboard Subsystem (`dashboard/`)

Web-based control panel with real-time telemetry.

| File | Purpose |
|------|---------|
| `server.py` | FastAPI + WebSocket server (telemetry, camera, commands, mode switching) |
| `static/index.html` | Dashboard UI with camera feed, state display, controls, joystick |
| `static/dashboard.css` | Dark glassmorphism theme |
| `static/dashboard.js` | WebSocket client, IMU gauges, mode switcher, virtual joystick |

**API Endpoints:**

| Method | Endpoint | Purpose |
|--------|----------|---------|
| GET | `/api/status` | Full system telemetry |
| POST | `/api/command` | Natural language command |
| POST | `/api/action` | Direct action `{action, speed}` |
| POST | `/api/emergency_stop` | Trigger E-stop |
| GET | `/api/mode` | Get current control mode |
| POST | `/api/mode` | Switch control mode |
| POST | `/api/manual` | Submit manual command |

---

### 8. ROS2 Subsystem (`ros2_ws/src/robot_dog/`)

Full ROS2 integration for advanced deployments.

| File | Purpose |
|------|---------|
| `sensor_publisher.py` | IMU, camera, distance publishers |
| `motor_controller_node.py` | `cmd_vel` to servo subscriber |
| `ekf_node.py` | Extended Kalman Filter for sensor fusion |
| `brain_node.py` | LLM brain as a ROS2 node |
| `launch/robot_dog_launch.py` | Launch all nodes |

---

## Project Structure

```
Quadruped-Robot-AI-Test/
+-- config/
|   +-- robot_config.yaml          # Central configuration
+-- control/                        # Control Mode System (NEW)
|   +-- mode_manager.py            # Manual / Autonomous / Hybrid
|   +-- manual_controller.py       # Direct user input handler
|   +-- autonomous_controller.py   # LLM/vision pipeline wrapper
+-- hardware/                       # Hardware Abstraction Layer
|   +-- servo_driver.py            # PCA9685 servo controller
|   +-- imu_sensor.py              # MPU6050/BNO055 IMU
|   +-- distance_sensor.py         # Ultrasonic / LiDAR
|   +-- camera_module.py           # OpenCV camera capture
+-- locomotion/                     # Locomotion Engine
|   +-- kinematics.py              # Inverse kinematics (3-DOF)
|   +-- gait_controller.py         # Walk, trot, turn gaits
|   +-- body_controller.py         # Body pose manager
+-- brain/                          # LLM-Powered Brain
|   +-- llm_client.py              # OpenAI + Gemini API wrapper
|   +-- vision_processor.py        # Camera to LLM scene analysis
|   +-- decision_engine.py         # LLM to action mapping
|   +-- command_parser.py          # Natural language commands
|   +-- state_machine.py           # Robot behavior states
|   +-- safety_system.py           # Fall detection, e-stop
+-- communication/                  # Communication Layer
|   +-- serial_protocol.py         # Binary packet protocol
|   +-- serial_bridge.py           # PySerial bridge
+-- firmware/
|   +-- arduino_firmware/
|       +-- arduino_firmware.ino   # Arduino/ESP32 motor control
+-- ros2_ws/src/robot_dog/         # ROS2 Package
|   +-- sensor_publisher.py        # IMU/camera/distance publishers
|   +-- motor_controller_node.py   # cmd_vel to servo subscriber
|   +-- ekf_node.py                # Extended Kalman Filter
|   +-- brain_node.py              # LLM brain ROS2 wrapper
|   +-- launch/
|       +-- robot_dog_launch.py    # Launch all nodes
+-- dashboard/                      # Web Dashboard
|   +-- server.py                  # FastAPI + WebSocket server
|   +-- static/
|       +-- index.html             # Dashboard UI
|       +-- dashboard.css          # Dark glassmorphism styles
|       +-- dashboard.js           # WebSocket client + controls
+-- tests/                          # Unit & integration tests
+-- main.py                         # System entry point
+-- requirements.txt
+-- README.md
```

## Quick Start

### 1. Clone & Install
```bash
git clone https://github.com/Sharan-G-S/Quadruped-Robot-Brain-Test.git
cd Quadruped-Robot-Brain-Test
pip install -r requirements.txt
```

### 2. Configure
Edit `config/robot_config.yaml`:
- Set your **LLM API key** (OpenAI or Gemini)
- Choose **mode**: `simulation`, `serial`, or `ros2`
- Choose **control mode**: `manual`, `autonomous`, or `hybrid`
- Adjust servo calibration for your hardware

### 3. Run
```bash
# Autonomous mode (default -- AI drives the robot)
python main.py --mode simulation --control-mode autonomous

# Manual mode (you control via dashboard/keyboard)
python main.py --mode simulation --control-mode manual

# Hybrid mode (manual priority, AI fallback)
python main.py --mode simulation --control-mode hybrid

# Serial mode (Arduino/ESP32 connected)
python main.py --mode serial --control-mode autonomous

# ROS2 mode (full ROS2 stack)
cd ros2_ws && colcon build
ros2 launch robot_dog robot_dog_launch.py
```

### 4. Dashboard
Open `http://localhost:8080` for the live control panel.
- Use the **mode switcher** in the header to toggle between control modes
- Use the **virtual joystick** for manual control (manual/hybrid mode)
- Use **keyboard shortcuts**: W/A/S/D = move, R = trot, Space = stand, Esc = E-stop

## Hardware Requirements

| Component | Example Part | Purpose |
|-----------|-------------|---------|
| SBC | Raspberry Pi 4/5, Jetson Nano | Main compute |
| MCU | Arduino Mega / ESP32 | Real-time servo control |
| Servos x12 | MG996R / DS3218 | 3 DOF x 4 legs |
| Servo Board | PCA9685 (I2C) | 16-ch PWM driver |
| IMU | MPU6050 / BNO055 | Orientation sensing |
| Camera | USB webcam / RPi Cam | Vision input |
| Distance | HC-SR04 ultrasonic | Obstacle detection |
| Power | 2S/3S LiPo battery | Power supply |
| Frame | 3D-printed / metal kit | Robot chassis |

## Modes

| Mode | Description | Hardware Required |
|------|-------------|-------------------|
| `simulation` | Mock sensors, visual dashboard | None |
| `serial` | Direct serial to Arduino/ESP32 | MCU + Servos |
| `ros2` | Full ROS2 stack with EKF | All sensors |

---

<p align="center">
  <em>Made by Sharan G S</em>
</p>
