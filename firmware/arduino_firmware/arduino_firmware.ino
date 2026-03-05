/*
 * arduino_firmware.ino — QuadBot-AI Motor Controller Firmware
 * ============================================================
 * Runs on Arduino Mega / ESP32 to receive servo commands via serial
 * from the onboard SBC (Raspberry Pi / Jetson) and drive servos
 * through PCA9685 I2C PWM board.
 *
 * Also reads IMU (MPU6050) and sends sensor data back.
 *
 * Serial Protocol:
 *   [0xAA] [CMD] [LENGTH] [DATA...] [CHECKSUM]
 *   Checksum = XOR of CMD + LENGTH + DATA bytes
 *
 * Commands:
 *   0x01 SERVO_WRITE:  [channel(1), angle_x10(2)]
 *   0x02 SERVO_MULTI:  [count(1), (ch(1)+angle_x10(2)) × N]
 *   0x03 SENSOR_REQ:   [sensor_id(1)]
 *   0x05 HEARTBEAT:    [timestamp(4)]
 *   0xFF EMERGENCY_STOP: (no data)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ── Configuration ──────────────────────────────────────────────

#define SERIAL_BAUD    115200
#define START_BYTE     0xAA
#define MAX_PACKET_LEN 64

// PCA9685 servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo calibration
#define SERVO_MIN_PULSE  125   // ~500μs  at 50Hz
#define SERVO_MAX_PULSE  625   // ~2500μs at 50Hz
#define SERVO_FREQ       50    // 50Hz for standard servos

// Command types
#define CMD_SERVO_WRITE    0x01
#define CMD_SERVO_MULTI    0x02
#define CMD_SENSOR_REQ     0x03
#define CMD_SENSOR_DATA    0x04
#define CMD_HEARTBEAT      0x05
#define CMD_STATUS         0x06
#define CMD_EMERGENCY_STOP 0xFF

// Sensor IDs
#define SENSOR_IMU       0x01
#define SENSOR_DISTANCE  0x02
#define SENSOR_BATTERY   0x03
#define SENSOR_ALL       0xFF

// ── State ──────────────────────────────────────────────────────

uint8_t rxBuffer[MAX_PACKET_LEN];
int rxIndex = 0;
bool receivingPacket = false;
int expectedLength = 0;

bool emergencyStop = false;
unsigned long lastHeartbeat = 0;
unsigned long heartbeatTimeout = 5000;  // 5 seconds

// Current servo angles (for telemetry)
float servoAngles[16];

// ── Setup ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) { delay(10); }

    // Initialize PCA9685
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);

    // Initialize I2C for IMU
    Wire.begin();

    // Center all servos
    for (int i = 0; i < 12; i++) {
        setServoAngle(i, 90.0);
        servoAngles[i] = 90.0;
    }

    delay(500);
    Serial.println("QuadBot-AI Firmware Ready");

    lastHeartbeat = millis();
}

// ── Main Loop ──────────────────────────────────────────────────

void loop() {
    // Read serial data
    while (Serial.available() > 0) {
        uint8_t b = Serial.read();
        processSerialByte(b);
    }

    // Check heartbeat timeout (safety)
    if (millis() - lastHeartbeat > heartbeatTimeout && !emergencyStop) {
        // Lost communication — emergency stop
        emergencyStopAction();
    }

    delay(1);
}

// ── Serial Packet Parser ───────────────────────────────────────

void processSerialByte(uint8_t b) {
    if (!receivingPacket) {
        if (b == START_BYTE) {
            receivingPacket = true;
            rxIndex = 0;
        }
        return;
    }

    rxBuffer[rxIndex++] = b;

    // We have CMD + LENGTH after 2 bytes
    if (rxIndex == 2) {
        expectedLength = rxBuffer[1];  // LENGTH field
    }

    // Full packet received: CMD(1) + LENGTH(1) + DATA(expectedLength) + CHECKSUM(1)
    if (rxIndex >= 3 + expectedLength) {
        // Verify checksum
        uint8_t checksum = 0;
        for (int i = 0; i < 2 + expectedLength; i++) {
            checksum ^= rxBuffer[i];
        }

        if (checksum == rxBuffer[2 + expectedLength]) {
            handlePacket(rxBuffer[0], &rxBuffer[2], expectedLength);
        }

        receivingPacket = false;
        rxIndex = 0;
    }

    // Overflow protection
    if (rxIndex >= MAX_PACKET_LEN) {
        receivingPacket = false;
        rxIndex = 0;
    }
}

// ── Command Handler ────────────────────────────────────────────

void handlePacket(uint8_t cmd, uint8_t* data, uint8_t length) {
    switch (cmd) {
        case CMD_SERVO_WRITE:
            if (length >= 3 && !emergencyStop) {
                uint8_t channel = data[0];
                uint16_t angle_x10 = data[1] | (data[2] << 8);
                float angle = angle_x10 / 10.0;
                setServoAngle(channel, angle);
                servoAngles[channel] = angle;
            }
            break;

        case CMD_SERVO_MULTI:
            if (length >= 1 && !emergencyStop) {
                uint8_t count = data[0];
                for (int i = 0; i < count && (1 + i * 3 + 2) < length; i++) {
                    uint8_t ch = data[1 + i * 3];
                    uint16_t ang_x10 = data[2 + i * 3] | (data[3 + i * 3] << 8);
                    float angle = ang_x10 / 10.0;
                    if (ch < 16) {
                        setServoAngle(ch, angle);
                        servoAngles[ch] = angle;
                    }
                }
            }
            break;

        case CMD_SENSOR_REQ:
            if (length >= 1) {
                uint8_t sensorId = data[0];
                sendSensorData(sensorId);
            }
            break;

        case CMD_HEARTBEAT:
            lastHeartbeat = millis();
            // Echo heartbeat back
            sendHeartbeatAck();
            break;

        case CMD_EMERGENCY_STOP:
            emergencyStopAction();
            break;

        default:
            break;
    }
}

// ── Servo Control ──────────────────────────────────────────────

void setServoAngle(uint8_t channel, float angle) {
    if (channel >= 16) return;
    angle = constrain(angle, 0.0, 180.0);
    uint16_t pulse = map(angle * 10, 0, 1800, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);
}

void relaxServos() {
    for (int i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 0);  // Turn off PWM
    }
}

// ── Emergency Stop ─────────────────────────────────────────────

void emergencyStopAction() {
    emergencyStop = true;
    relaxServos();

    // Send status back
    uint8_t statusData[] = {0x01};  // Emergency flag
    sendPacket(CMD_STATUS, statusData, 1);
}

// ── Sensor Data ────────────────────────────────────────────────

void sendSensorData(uint8_t sensorId) {
    if (sensorId == SENSOR_IMU || sensorId == SENSOR_ALL) {
        // Read MPU6050 (basic read)
        int16_t imuData[6];
        readMPU6050(imuData);

        uint8_t payload[13];
        payload[0] = SENSOR_IMU;
        memcpy(&payload[1], imuData, 12);
        sendPacket(CMD_SENSOR_DATA, payload, 13);
    }

    if (sensorId == SENSOR_BATTERY || sensorId == SENSOR_ALL) {
        // Read battery voltage from analog pin A0
        int rawADC = analogRead(A0);
        uint16_t voltage_x100 = (uint16_t)(rawADC * (5.0 / 1023.0) * 3.0 * 100);
        // Assumes voltage divider: R1=20k, R2=10k → factor 3

        uint8_t payload[3];
        payload[0] = SENSOR_BATTERY;
        payload[1] = voltage_x100 & 0xFF;
        payload[2] = (voltage_x100 >> 8) & 0xFF;
        sendPacket(CMD_SENSOR_DATA, payload, 3);
    }
}

void readMPU6050(int16_t* data) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);  // Start at ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)14, (uint8_t)true);

    // Accelerometer (3 axes)
    data[0] = (Wire.read() << 8) | Wire.read();  // accel_x
    data[1] = (Wire.read() << 8) | Wire.read();  // accel_y
    data[2] = (Wire.read() << 8) | Wire.read();  // accel_z

    // Skip temperature (2 bytes)
    Wire.read(); Wire.read();

    // Gyroscope (3 axes)
    data[3] = (Wire.read() << 8) | Wire.read();  // gyro_x
    data[4] = (Wire.read() << 8) | Wire.read();  // gyro_y
    data[5] = (Wire.read() << 8) | Wire.read();  // gyro_z
}

// ── Packet Sending ─────────────────────────────────────────────

void sendPacket(uint8_t cmd, uint8_t* data, uint8_t length) {
    uint8_t checksum = cmd ^ length;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }

    Serial.write(START_BYTE);
    Serial.write(cmd);
    Serial.write(length);
    Serial.write(data, length);
    Serial.write(checksum);
}

void sendHeartbeatAck() {
    uint32_t ts = millis();
    uint8_t data[4];
    memcpy(data, &ts, 4);
    sendPacket(CMD_HEARTBEAT, data, 4);
}
