"""
serial_protocol.py — Binary Packet Protocol for Serial Communication
======================================================================
Defines the packet structure for communicating between the SBC (RPi/Jetson)
and the microcontroller (Arduino/ESP32).

Packet format:
  [START_BYTE (0xAA)] [CMD_TYPE (1B)] [LENGTH (1B)] [DATA (N bytes)] [CHECKSUM (1B)]

Checksum = XOR of all bytes from CMD_TYPE through DATA.
"""

import logging
import struct
from typing import Optional, Tuple, List
from enum import IntEnum

logger = logging.getLogger(__name__)

START_BYTE = 0xAA


class CommandType(IntEnum):
    """Command types for the serial protocol."""
    SERVO_WRITE = 0x01       # Set single servo: [channel(1), angle_x10(2)]
    SERVO_MULTI = 0x02       # Set multiple servos: [count(1), (ch(1)+angle_x10(2)) × N]
    SENSOR_REQUEST = 0x03    # Request sensor data: [sensor_id(1)]
    SENSOR_DATA = 0x04       # Sensor data response: [sensor_id(1), data(N)]
    HEARTBEAT = 0x05         # Heartbeat: [timestamp(4)]
    STATUS = 0x06            # Status response: [flags(1)]
    SET_MODE = 0x07          # Set controller mode: [mode(1)]
    EMERGENCY_STOP = 0xFF    # Emergency stop (no data)


class SensorID(IntEnum):
    """Sensor identifiers for the serial protocol."""
    IMU = 0x01
    DISTANCE = 0x02
    BATTERY = 0x03
    ALL = 0xFF


class SerialProtocol:
    """Binary packet encoder/decoder for serial communication."""

    @staticmethod
    def compute_checksum(data: bytes) -> int:
        """XOR checksum of all bytes."""
        cs = 0
        for b in data:
            cs ^= b
        return cs

    # ── Encoding (Python → Arduino) ─────────────────────────────

    @staticmethod
    def encode_servo_write(channel: int, angle: float) -> bytes:
        """Encode a single servo write command."""
        angle_x10 = int(angle * 10)  # Transmit with 0.1° precision
        data = struct.pack("BH", channel, angle_x10)
        return SerialProtocol._build_packet(CommandType.SERVO_WRITE, data)

    @staticmethod
    def encode_servo_multi(angles: dict) -> bytes:
        """
        Encode a multi-servo write command.
        angles: {channel: angle, ...}
        """
        count = len(angles)
        data = struct.pack("B", count)
        for ch, ang in angles.items():
            data += struct.pack("BH", ch, int(ang * 10))
        return SerialProtocol._build_packet(CommandType.SERVO_MULTI, data)

    @staticmethod
    def encode_sensor_request(sensor_id: int) -> bytes:
        """Request sensor data from the microcontroller."""
        data = struct.pack("B", sensor_id)
        return SerialProtocol._build_packet(CommandType.SENSOR_REQUEST, data)

    @staticmethod
    def encode_heartbeat(timestamp: int = 0) -> bytes:
        """Encode a heartbeat packet."""
        data = struct.pack("<I", timestamp & 0xFFFFFFFF)
        return SerialProtocol._build_packet(CommandType.HEARTBEAT, data)

    @staticmethod
    def encode_emergency_stop() -> bytes:
        """Encode an emergency stop command."""
        return SerialProtocol._build_packet(CommandType.EMERGENCY_STOP, b"")

    @staticmethod
    def _build_packet(cmd: int, data: bytes) -> bytes:
        """Build a complete packet with start byte, header, data, and checksum."""
        length = len(data)
        payload = struct.pack("BB", cmd, length) + data
        checksum = SerialProtocol.compute_checksum(payload)
        return bytes([START_BYTE]) + payload + bytes([checksum])

    # ── Decoding (Arduino → Python) ─────────────────────────────

    @staticmethod
    def decode_packet(raw: bytes) -> Optional[Tuple[int, bytes]]:
        """
        Decode a raw packet.

        Returns:
            (command_type, data_bytes) or None if invalid.
        """
        if len(raw) < 4:  # Minimum: START + CMD + LEN + CHECKSUM
            return None

        if raw[0] != START_BYTE:
            logger.debug(f"Invalid start byte: 0x{raw[0]:02X}")
            return None

        cmd = raw[1]
        length = raw[2]

        if len(raw) < 4 + length:
            logger.debug(f"Packet too short: expected {4 + length}, got {len(raw)}")
            return None

        data = raw[3:3 + length]
        checksum = raw[3 + length]

        # Verify checksum
        payload = raw[1:3 + length]
        expected_cs = SerialProtocol.compute_checksum(payload)
        if checksum != expected_cs:
            logger.debug(
                f"Checksum mismatch: got 0x{checksum:02X}, "
                f"expected 0x{expected_cs:02X}"
            )
            return None

        return (cmd, data)

    @staticmethod
    def decode_sensor_data(cmd: int, data: bytes) -> Optional[dict]:
        """Decode sensor data response packet."""
        if cmd != CommandType.SENSOR_DATA or len(data) < 1:
            return None

        sensor_id = data[0]
        payload = data[1:]

        if sensor_id == SensorID.IMU and len(payload) >= 12:
            # 3 × int16 (accel) + 3 × int16 (gyro)
            values = struct.unpack("<6h", payload[:12])
            return {
                "sensor": "imu",
                "accel_x": values[0] / 100.0,
                "accel_y": values[1] / 100.0,
                "accel_z": values[2] / 100.0,
                "gyro_x": values[3] / 100.0,
                "gyro_y": values[4] / 100.0,
                "gyro_z": values[5] / 100.0,
            }

        elif sensor_id == SensorID.DISTANCE and len(payload) >= 2:
            distance = struct.unpack("<H", payload[:2])[0]
            return {"sensor": "distance", "distance_cm": distance}

        elif sensor_id == SensorID.BATTERY and len(payload) >= 2:
            voltage = struct.unpack("<H", payload[:2])[0] / 100.0
            return {"sensor": "battery", "voltage": voltage}

        return {"sensor": "unknown", "raw": payload.hex()}

    # ── Stream Parser ───────────────────────────────────────────

    class StreamParser:
        """
        Stateful parser for a continuous byte stream.
        Finds and extracts complete packets from incoming data.
        """

        def __init__(self):
            self._buffer = bytearray()

        def feed(self, data: bytes) -> List[Tuple[int, bytes]]:
            """
            Feed raw bytes and return list of decoded (cmd, data) packets.
            """
            self._buffer.extend(data)
            packets = []

            while len(self._buffer) >= 4:
                # Find start byte
                idx = self._buffer.find(START_BYTE)
                if idx < 0:
                    self._buffer.clear()
                    break
                if idx > 0:
                    self._buffer = self._buffer[idx:]

                if len(self._buffer) < 3:
                    break

                length = self._buffer[2]
                packet_size = 4 + length

                if len(self._buffer) < packet_size:
                    break

                raw = bytes(self._buffer[:packet_size])
                self._buffer = self._buffer[packet_size:]

                result = SerialProtocol.decode_packet(raw)
                if result:
                    packets.append(result)

            return packets
