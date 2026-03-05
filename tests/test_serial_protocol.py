"""
test_serial_protocol.py — Unit Tests for Serial Protocol
==========================================================
"""

import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from communication.serial_protocol import SerialProtocol, CommandType, SensorID


class TestSerialProtocol:
    """Test packet encoding, decoding, and stream parsing."""

    def test_encode_servo_write(self):
        """Servo write packet should have correct structure."""
        packet = SerialProtocol.encode_servo_write(3, 90.0)
        assert packet[0] == 0xAA  # Start byte
        assert packet[1] == CommandType.SERVO_WRITE
        assert len(packet) >= 4

    def test_encode_decode_servo_write(self):
        """Encode then decode should preserve data."""
        packet = SerialProtocol.encode_servo_write(5, 120.5)
        result = SerialProtocol.decode_packet(packet)
        assert result is not None
        cmd, data = result
        assert cmd == CommandType.SERVO_WRITE
        assert data[0] == 5  # Channel

    def test_encode_servo_multi(self):
        """Multi-servo packet should handle multiple channels."""
        angles = {0: 90.0, 1: 45.0, 2: 135.0}
        packet = SerialProtocol.encode_servo_multi(angles)
        result = SerialProtocol.decode_packet(packet)
        assert result is not None
        cmd, data = result
        assert cmd == CommandType.SERVO_MULTI
        assert data[0] == 3  # Count

    def test_encode_heartbeat(self):
        """Heartbeat packet should encode timestamp."""
        packet = SerialProtocol.encode_heartbeat(12345)
        result = SerialProtocol.decode_packet(packet)
        assert result is not None
        cmd, data = result
        assert cmd == CommandType.HEARTBEAT
        assert len(data) == 4

    def test_encode_emergency_stop(self):
        """Emergency stop should be a zero-data packet."""
        packet = SerialProtocol.encode_emergency_stop()
        result = SerialProtocol.decode_packet(packet)
        assert result is not None
        cmd, data = result
        assert cmd == CommandType.EMERGENCY_STOP
        assert len(data) == 0

    def test_checksum_validation(self):
        """Corrupted packet should fail decoding."""
        packet = bytearray(SerialProtocol.encode_servo_write(0, 90.0))
        packet[-1] ^= 0xFF  # Corrupt checksum
        result = SerialProtocol.decode_packet(bytes(packet))
        assert result is None

    def test_invalid_start_byte(self):
        """Packet with wrong start byte should fail."""
        packet = bytearray(SerialProtocol.encode_servo_write(0, 90.0))
        packet[0] = 0x00  # Wrong start byte
        result = SerialProtocol.decode_packet(bytes(packet))
        assert result is None

    def test_stream_parser(self):
        """Stream parser should extract multiple packets from a byte stream."""
        parser = SerialProtocol.StreamParser()

        p1 = SerialProtocol.encode_servo_write(0, 90.0)
        p2 = SerialProtocol.encode_heartbeat(100)
        p3 = SerialProtocol.encode_emergency_stop()

        # Feed all three packets as one continuous stream
        packets = parser.feed(p1 + p2 + p3)
        assert len(packets) == 3
        assert packets[0][0] == CommandType.SERVO_WRITE
        assert packets[1][0] == CommandType.HEARTBEAT
        assert packets[2][0] == CommandType.EMERGENCY_STOP

    def test_stream_parser_partial(self):
        """Stream parser should handle partial packets."""
        parser = SerialProtocol.StreamParser()
        packet = SerialProtocol.encode_servo_write(0, 90.0)

        # Feed first half
        packets = parser.feed(packet[:3])
        assert len(packets) == 0  # Incomplete

        # Feed second half
        packets = parser.feed(packet[3:])
        assert len(packets) == 1  # Now complete

    def test_sensor_data_decode(self):
        """Sensor data decoding should work for known formats."""
        import struct

        # Distance sensor data
        data = bytes([SensorID.DISTANCE]) + struct.pack("<H", 150)
        result = SerialProtocol.decode_sensor_data(CommandType.SENSOR_DATA, data)
        assert result is not None
        assert result["sensor"] == "distance"
        assert result["distance_cm"] == 150
