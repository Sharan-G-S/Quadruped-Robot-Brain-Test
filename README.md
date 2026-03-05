<p align="center">
  <h1 align="center">QuadBot-AI -- Intelligent Robot Dog</h1>
  <p align="center">
    <strong>LLM-Powered Quadruped Robot with Computer Vision, ROS2, and EKF Sensor Fusion</strong>
  </p>
  <p align="center">
    <em>Built by Sharan G S</em>
  </p>
</p>

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        QuadBot-AI Brain                         │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌─────────────┐   ┌──────────────┐   ┌─────────────────────┐ │
│   │  LLM Vision │   │   Decision   │   │   State Machine     │ │
│   │  (GPT-4o /  │──▶│   Engine     │──▶│   & Behavior        │ │
│   │   Gemini)   │   │              │   │   Controller        │ │
│   └──────┬──────┘   └──────────────┘   └──────────┬──────────┘ │
│          │                                         │             │
│   ┌──────┴──────┐                        ┌────────▼──────────┐ │
│   │   Camera    │                        │  Safety System    │ │
│   │   Module    │                        │  (Fall Detect,    │ │
│   │  (OpenCV)   │                        │   E-Stop)         │ │
│   └─────────────┘                        └───────────────────┘ │
│                                                                  │
├───────────────────────── LOCOMOTION ─────────────────────────────┤
│                                                                  │
│   ┌─────────────┐   ┌──────────────┐   ┌─────────────────────┐ │
│   │  Inverse    │   │    Gait      │   │   Body Pose         │ │
│   │  Kinematics │◀──│  Controller  │◀──│   Controller        │ │
│   │  (3-DOF)    │   │ (Walk/Trot)  │   │                     │ │
│   └──────┬──────┘   └──────────────┘   └─────────────────────┘ │
│          │                                                       │
├──────────┼──────────── COMMUNICATION ────────────────────────────┤
│          │                                                       │
│   ┌──────▼──────┐   ┌──────────────┐   ┌─────────────────────┐ │
│   │   Serial    │   │    ROS2      │   │   EKF Sensor        │ │
│   │   Bridge    │   │   Nodes      │   │   Fusion            │ │
│   │ (Arduino)   │   │              │   │                     │ │
│   └──────┬──────┘   └──────┬───────┘   └─────────────────────┘ │
│          │                 │                                     │
├──────────┼─────────────────┼──── HARDWARE ───────────────────────┤
│          │                 │                                     │
│   ┌──────▼──────┐   ┌─────▼────────┐   ┌─────────────────────┐ │
│   │  12× Servos │   │  IMU Sensor  │   │  Distance Sensor    │ │
│   │  (PCA9685)  │   │  (MPU6050)   │   │  (Ultrasonic)       │ │
│   └─────────────┘   └──────────────┘   └─────────────────────┘ │
│                                                                  │
├───────────────────────── DASHBOARD ──────────────────────────────┤
│   FastAPI + WebSocket │ Live Camera │ Telemetry │ Controls      │
└──────────────────────────────────────────────────────────────────┘
```

## Project Structure

```
Quadruped-Robot-AI-Test/
├── config/
│   └── robot_config.yaml          # Central configuration
├── hardware/                       # Hardware Abstraction Layer
│   ├── servo_driver.py            # PCA9685 servo controller
│   ├── imu_sensor.py              # MPU6050/BNO055 IMU
│   ├── distance_sensor.py         # Ultrasonic / LiDAR
│   └── camera_module.py           # OpenCV camera capture
├── locomotion/                     # Locomotion Engine
│   ├── kinematics.py              # Inverse kinematics (3-DOF)
│   ├── gait_controller.py         # Walk, trot, turn gaits
│   └── body_controller.py         # Body pose manager
├── brain/                          # LLM-Powered Brain
│   ├── llm_client.py              # OpenAI + Gemini API wrapper
│   ├── vision_processor.py        # Camera → LLM scene analysis
│   ├── decision_engine.py         # LLM → action mapping
│   ├── command_parser.py          # Natural language commands
│   ├── state_machine.py           # Robot behavior states
│   └── safety_system.py           # Fall detection, e-stop
├── communication/                  # Communication Layer
│   ├── serial_protocol.py         # Binary packet protocol
│   └── serial_bridge.py           # PySerial bridge
├── firmware/
│   └── arduino_firmware/
│       └── arduino_firmware.ino   # Arduino/ESP32 motor control
├── ros2_ws/src/robot_dog/         # ROS2 Package
│   ├── sensor_publisher.py        # IMU/camera/distance publishers
│   ├── motor_controller_node.py   # cmd_vel → servo subscriber
│   ├── ekf_node.py                # Extended Kalman Filter
│   ├── brain_node.py              # LLM brain ROS2 wrapper
│   └── launch/
│       └── robot_dog_launch.py    # Launch all nodes
├── dashboard/                      # Web Dashboard
│   ├── server.py                  # FastAPI + WebSocket server
│   └── static/
│       ├── index.html             # Dashboard UI
│       ├── dashboard.css          # Dark glassmorphism styles
│       └── dashboard.js           # WebSocket client + controls
├── tests/                          # Unit & integration tests
├── main.py                         # System entry point
├── requirements.txt
└── README.md
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
- Adjust servo calibration for your hardware

### 3. Run
```bash
# Simulation mode (no hardware needed)
python main.py --mode simulation

# Serial mode (Arduino/ESP32 connected)
python main.py --mode serial

# ROS2 mode (full ROS2 stack)
cd ros2_ws && colcon build
ros2 launch robot_dog robot_dog_launch.py
```

### 4. Dashboard
Open `http://localhost:8080` for the live control panel.

## Hardware Requirements

| Component | Example Part | Purpose |
|-----------|-------------|---------|
| SBC | Raspberry Pi 4/5, Jetson Nano | Main compute |
| MCU | Arduino Mega / ESP32 | Real-time servo control |
| Servos ×12 | MG996R / DS3218 | 3 DOF × 4 legs |
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

## Communication Protocol

Serial packets: `[0xAA, CMD, LENGTH, DATA..., CHECKSUM]`

| CMD | Name | Data |
|-----|------|------|
| 0x01 | SERVO_WRITE | channel (1B) + angle (2B) |
| 0x02 | SERVO_MULTI | count + [ch+angle] × N |
| 0x03 | SENSOR_REQ | sensor_id (1B) |
| 0x04 | SENSOR_DATA | sensor_id + data |
| 0x05 | HEARTBEAT | timestamp (4B) |
| 0xFF | E_STOP | — |

---

<p align="center">
  <em>Made by Sharan G S</em>
</p>
