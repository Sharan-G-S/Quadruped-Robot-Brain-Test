"""
imu_sensor.py — MPU6050 / BNO055 / Simulation IMU
===================================================
Reads orientation (roll, pitch, yaw) and acceleration with complementary filter.
"""

import logging
import math
import time
import random
from typing import Dict, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class IMUSensor:
    """IMU reader with complementary filter and fall detection."""

    def __init__(self, config: dict):
        self.sensor_type = config.get("type", "simulation")
        self.i2c_address = config.get("i2c_address", 0x68)
        self.alpha = config.get("complementary_alpha", 0.96)
        self.simulation = False

        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._accel = np.zeros(3)
        self._gyro = np.zeros(3)
        self._last_time = time.time()
        self._sim_t = 0.0

        self._init_sensor()

    def _init_sensor(self):
        if self.sensor_type == "simulation":
            self.simulation = True
            logger.info("📐 IMU: simulation mode")
            return
        try:
            if self.sensor_type == "mpu6050":
                import adafruit_mpu6050, board, busio
                i2c = busio.I2C(board.SCL, board.SDA)
                self.sensor = adafruit_mpu6050.MPU6050(i2c, address=self.i2c_address)
                logger.info("MPU6050 initialized")
            elif self.sensor_type == "bno055":
                import adafruit_bno055, board, busio
                i2c = busio.I2C(board.SCL, board.SDA)
                self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=self.i2c_address)
                logger.info("BNO055 initialized")
            else:
                raise ValueError(f"Unknown IMU: {self.sensor_type}")
        except Exception as e:
            logger.warning(f"IMU init failed ({e}), using simulation")
            self.simulation = True

    # ── Read ────────────────────────────────────────────────────

    def read(self) -> Dict[str, float]:
        """Return latest IMU data dict."""
        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        if self.simulation:
            self._read_sim(dt)
        else:
            self._read_hw(dt)

        return {
            "roll": self._roll, "pitch": self._pitch, "yaw": self._yaw,
            "accel_x": float(self._accel[0]),
            "accel_y": float(self._accel[1]),
            "accel_z": float(self._accel[2]),
            "gyro_x": float(self._gyro[0]),
            "gyro_y": float(self._gyro[1]),
            "gyro_z": float(self._gyro[2]),
            "timestamp": now,
        }

    def _read_sim(self, dt: float):
        self._sim_t += dt
        self._roll = 3.0 * math.sin(self._sim_t * 2.5) + random.gauss(0, 0.3)
        self._pitch = 2.0 * math.sin(self._sim_t * 1.8 + 0.5) + random.gauss(0, 0.3)
        self._yaw += random.gauss(0, 0.1)
        self._accel = np.array([random.gauss(0, 0.2), random.gauss(0, 0.2),
                                9.81 + random.gauss(0, 0.1)])
        self._gyro = np.array([math.radians(self._roll) * 0.1 + random.gauss(0, 0.01),
                               math.radians(self._pitch) * 0.1 + random.gauss(0, 0.01),
                               random.gauss(0, 0.005)])

    def _read_hw(self, dt: float):
        try:
            if self.sensor_type == "mpu6050":
                accel = self.sensor.acceleration
                gyro = self.sensor.gyro
                self._accel = np.array(accel)
                self._gyro = np.array(gyro)

                accel_roll = math.degrees(math.atan2(accel[1], accel[2]))
                accel_pitch = math.degrees(
                    math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
                )
                self._roll = self.alpha * (self._roll + math.degrees(gyro[0]) * dt) \
                             + (1 - self.alpha) * accel_roll
                self._pitch = self.alpha * (self._pitch + math.degrees(gyro[1]) * dt) \
                              + (1 - self.alpha) * accel_pitch
                self._yaw += math.degrees(gyro[2]) * dt

            elif self.sensor_type == "bno055":
                euler = self.sensor.euler
                if euler[0] is not None:
                    self._yaw, self._roll, self._pitch = euler
                accel = self.sensor.acceleration
                gyro = self.sensor.gyro
                if accel:
                    self._accel = np.array(accel)
                if gyro:
                    self._gyro = np.array(gyro)
        except Exception as e:
            logger.error(f"IMU read error: {e}")

    # ── Properties ──────────────────────────────────────────────

    @property
    def orientation(self) -> Tuple[float, float, float]:
        return (self._roll, self._pitch, self._yaw)

    @property
    def acceleration_magnitude(self) -> float:
        return float(np.linalg.norm(self._accel))

    def is_falling(self, threshold_g: float = 3.0) -> bool:
        return self.acceleration_magnitude > threshold_g * 9.81

    def is_tilted(self, max_deg: float = 45.0) -> bool:
        return abs(self._roll) > max_deg or abs(self._pitch) > max_deg

    def get_telemetry(self) -> dict:
        return {
            "type": self.sensor_type, "simulation": self.simulation,
            "roll": round(self._roll, 2), "pitch": round(self._pitch, 2),
            "yaw": round(self._yaw, 2),
            "accel_mag": round(self.acceleration_magnitude, 2),
        }
