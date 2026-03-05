"""
distance_sensor.py — Ultrasonic / LiDAR / Simulation
=====================================================
Reads obstacle distance in centimeters with rolling-average filter.
"""

import logging
import math
import time
import random

logger = logging.getLogger(__name__)


class DistanceSensor:
    """Distance sensor with HC-SR04, TFMini LiDAR, or simulation support."""

    def __init__(self, config: dict):
        self.sensor_type = config.get("type", "simulation")
        self.trigger_pin = config.get("trigger_pin", 23)
        self.echo_pin = config.get("echo_pin", 24)
        self.max_dist = config.get("max_distance_cm", 400)
        self.min_dist = config.get("min_distance_cm", 2)
        self.threshold = config.get("obstacle_threshold_cm", 25)
        self.simulation = False

        self._readings: list = []
        self._filter_size = 5
        self._last = self.max_dist
        self._sim_t = 0.0

        self._init_sensor()

    def _init_sensor(self):
        if self.sensor_type == "simulation":
            self.simulation = True
            logger.info("📏 Distance sensor: simulation mode")
            return
        try:
            if self.sensor_type == "ultrasonic":
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.trigger_pin, GPIO.OUT)
                GPIO.setup(self.echo_pin, GPIO.IN)
                GPIO.output(self.trigger_pin, False)
                time.sleep(0.1)
                self.GPIO = GPIO
                logger.info("✅ HC-SR04 ultrasonic initialized")
            elif self.sensor_type == "lidar":
                import serial
                self.serial_conn = serial.Serial("/dev/ttyAMA0", 115200, timeout=0.1)
                logger.info("✅ TFMini LiDAR initialized")
            else:
                raise ValueError(f"Unknown sensor: {self.sensor_type}")
        except Exception as e:
            logger.warning(f"⚠️  Distance sensor failed ({e}), using simulation")
            self.simulation = True

    def read(self) -> float:
        """Read filtered distance in cm."""
        if self.simulation:
            raw = self._sim_read()
        elif self.sensor_type == "ultrasonic":
            raw = self._ultra_read()
        elif self.sensor_type == "lidar":
            raw = self._lidar_read()
        else:
            raw = self.max_dist

        self._readings.append(raw)
        if len(self._readings) > self._filter_size:
            self._readings.pop(0)

        self._last = max(self.min_dist,
                         min(self.max_dist,
                             sum(self._readings) / len(self._readings)))
        return self._last

    def _sim_read(self) -> float:
        self._sim_t += 0.05
        base = 100 + 60 * math.sin(self._sim_t * 0.3)
        if random.random() < 0.02:
            return random.uniform(10, 30)
        return max(self.min_dist, base + random.gauss(0, 2))

    def _ultra_read(self) -> float:
        try:
            G = self.GPIO
            G.output(self.trigger_pin, True)
            time.sleep(0.00001)
            G.output(self.trigger_pin, False)
            t0 = time.time()
            deadline = t0 + 0.04
            while G.input(self.echo_pin) == 0:
                t0 = time.time()
                if t0 > deadline:
                    return self.max_dist
            while G.input(self.echo_pin) == 1:
                t1 = time.time()
                if t1 > deadline:
                    return self.max_dist
            return (t1 - t0) * 34300 / 2
        except Exception as e:
            logger.error(f"Ultrasonic error: {e}")
            return self.max_dist

    def _lidar_read(self) -> float:
        try:
            data = self.serial_conn.read(9)
            if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
                return float(data[2] + data[3] * 256)
            return self._last
        except Exception as e:
            logger.error(f"LiDAR error: {e}")
            return self.max_dist

    @property
    def distance(self) -> float:
        return self._last

    def obstacle_detected(self) -> bool:
        return self._last < self.threshold

    def get_telemetry(self) -> dict:
        return {
            "type": self.sensor_type, "simulation": self.simulation,
            "distance_cm": round(self._last, 1),
            "obstacle": self.obstacle_detected(),
            "threshold_cm": self.threshold,
        }

    def cleanup(self):
        if not self.simulation and self.sensor_type == "ultrasonic":
            try:
                self.GPIO.cleanup()
            except Exception:
                pass
