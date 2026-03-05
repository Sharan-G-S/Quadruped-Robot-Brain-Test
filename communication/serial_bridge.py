"""
serial_bridge.py — PySerial Bridge to Arduino/ESP32
=====================================================
Manages the serial connection, sends servo commands, receives sensor data,
and handles reconnection and heartbeat.
"""

import logging
import time
import threading
from typing import Optional, Dict, Callable, List

from communication.serial_protocol import (
    SerialProtocol, CommandType, SensorID,
)

logger = logging.getLogger(__name__)


class SerialBridge:
    """
    Bidirectional serial bridge between Python (SBC) and Arduino/ESP32.
    Runs a background thread for receiving data.
    """

    def __init__(self, config: dict):
        self.port = config.get("port", "/dev/ttyUSB0")
        self.baud_rate = config.get("baud_rate", 115200)
        self.timeout = config.get("timeout", 0.1)
        self.heartbeat_interval = config.get("heartbeat_interval", 1.0)

        self.serial_conn = None
        self.connected = False
        self.simulation = False

        self._parser = SerialProtocol.StreamParser()
        self._sensor_callbacks: Dict[int, Callable] = {}
        self._last_heartbeat = 0
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._last_sensor_data: Dict[str, dict] = {}

        self._connect()

    # ── Connection ──────────────────────────────────────────────

    def _connect(self):
        try:
            import serial
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
            )
            time.sleep(2)  # Arduino resets on serial connect
            self.connected = True
            logger.info(f"✅ Serial connected: {self.port} @ {self.baud_rate}")
            self._start_rx_thread()
        except Exception as e:
            logger.warning(f"⚠️  Serial connection failed ({e}), using simulation")
            self.simulation = True
            self.connected = False

    def reconnect(self):
        """Attempt to reconnect."""
        self.stop()
        time.sleep(1)
        self._connect()

    # ── Sending Commands ────────────────────────────────────────

    def send_servo(self, channel: int, angle: float):
        """Send a single servo write command."""
        packet = SerialProtocol.encode_servo_write(channel, angle)
        self._send(packet)

    def send_servo_multi(self, angles: Dict[int, float]):
        """Send multiple servo angles in one packet."""
        packet = SerialProtocol.encode_servo_multi(angles)
        self._send(packet)

    def send_leg_angles(self, channels: List[int],
                        hip: float, upper: float, lower: float):
        """Send all 3 servo angles for a leg."""
        if len(channels) == 3:
            angles = {
                channels[0]: hip,
                channels[1]: upper,
                channels[2]: lower,
            }
            self.send_servo_multi(angles)

    def send_all_legs(self, servo_channels: dict,
                      angles: Dict[str, List[float]]):
        """
        Send angles for all legs at once.
        servo_channels: {"FR": [0,1,2], ...}
        angles: {"FR": [hip, upper, lower], ...}
        """
        all_angles = {}
        for leg, leg_angles in angles.items():
            chs = servo_channels.get(leg, [])
            for i, ch in enumerate(chs):
                if i < len(leg_angles):
                    all_angles[ch] = leg_angles[i]
        self.send_servo_multi(all_angles)

    def send_heartbeat(self):
        """Send heartbeat if interval has elapsed."""
        now = time.time()
        if now - self._last_heartbeat >= self.heartbeat_interval:
            packet = SerialProtocol.encode_heartbeat(int(now) & 0xFFFFFFFF)
            self._send(packet)
            self._last_heartbeat = now

    def send_emergency_stop(self):
        """Send emergency stop command."""
        packet = SerialProtocol.encode_emergency_stop()
        self._send(packet)
        logger.critical("🚨 Emergency stop sent via serial")

    def request_sensor(self, sensor_id: int = SensorID.ALL):
        """Request sensor data from the microcontroller."""
        packet = SerialProtocol.encode_sensor_request(sensor_id)
        self._send(packet)

    def _send(self, packet: bytes):
        if self.simulation:
            logger.debug(f"[SIM-SERIAL] TX: {packet.hex()}")
            return
        if not self.connected or not self.serial_conn:
            return
        try:
            self.serial_conn.write(packet)
        except Exception as e:
            logger.error(f"Serial write error: {e}")
            self.connected = False

    # ── Receiving Data ──────────────────────────────────────────

    def on_sensor_data(self, sensor_id: int, callback: Callable):
        """Register a callback for incoming sensor data."""
        self._sensor_callbacks[sensor_id] = callback

    def _start_rx_thread(self):
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="serial_rx"
        )
        self._rx_thread.start()

    def _rx_loop(self):
        """Background thread that reads and parses incoming serial data."""
        while self._running and self.connected:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    raw = self.serial_conn.read(self.serial_conn.in_waiting)
                    packets = self._parser.feed(raw)
                    for cmd, data in packets:
                        self._handle_rx_packet(cmd, data)
                else:
                    time.sleep(0.01)
            except Exception as e:
                logger.error(f"Serial RX error: {e}")
                time.sleep(0.1)

    def _handle_rx_packet(self, cmd: int, data: bytes):
        """Process a received packet."""
        if cmd == CommandType.SENSOR_DATA:
            sensor_data = SerialProtocol.decode_sensor_data(cmd, data)
            if sensor_data:
                sensor_name = sensor_data.get("sensor", "unknown")
                self._last_sensor_data[sensor_name] = sensor_data

                # Fire callback
                sensor_id = data[0] if data else 0
                cb = self._sensor_callbacks.get(sensor_id)
                if cb:
                    cb(sensor_data)

        elif cmd == CommandType.HEARTBEAT:
            logger.debug("Heartbeat ACK received")

        elif cmd == CommandType.STATUS:
            logger.debug(f"Status: {data.hex()}")

    # ── Lifecycle ───────────────────────────────────────────────

    def stop(self):
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=2)
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.connected = False
        logger.info("Serial bridge stopped")

    def get_telemetry(self) -> dict:
        return {
            "connected": self.connected,
            "simulation": self.simulation,
            "port": self.port,
            "baud_rate": self.baud_rate,
            "last_sensor_data": self._last_sensor_data,
        }
