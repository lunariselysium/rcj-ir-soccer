***

# Communication Protocol: STM32 <-> ROS 2 Bridge

This document describes the serial communication protocol used between the STM32 (Low-level Control) and the Raspberry Pi (High-level ROS 2).

## 1. Physical Layer
*   **Interface:** UART (TTL 3.3V)
*   **Baud Rate:** `115200` (Recommend `460800` or `921600` for high-rate IMU/IR data)
*   **Data Bits:** 8
*   **Parity:** None
*   **Stop Bits:** 1
*   **Flow Control:** None (DMA used on STM32)

## 2. Packet Framing & Encoding
To ensure data integrity and allow variable length messages, the protocol uses **COBS (Consistent Overhead Byte Stuffing)** with a **CRC8** checksum.

### 2.1 Wire Format
Data on the wire appears as a sequence of non-zero bytes terminated by a `0x00` (Null) byte.

| Sequence | Content | Description |
| :--- | :--- | :--- |
| **1** | `N` Bytes | **COBS Encoded Block** (Contains no `0x00` bytes) |
| **2** | `1` Byte | **0x00 Delimiter** (End of Packet) |

### 2.2 Decoded Packet Structure
After reading until `0x00` and applying `cobs_decode()`, the raw buffer results in:

| Offset | Field | Type | Description |
| :--- | :--- | :--- | :--- |
| 0 | **Packet ID** | `uint8_t` | Identifies the message type (see Section 3). |
| 1...N | **Payload** | `Mixed` | The actual data (sensor values, commands). |
| N+1 | **CRC8** | `uint8_t` | Checksum of `[ID + Payload]`. Poly: `0x07`. |

### 2.3 Data Types & Endianness
All multi-byte data is transmitted in **Little Endian** format (Standard for ARM Cortex-M and x86/ARM Linux).

*   `float`: IEEE 754 32-bit floating point.
*   `uint16_t`: Unsigned 16-bit integer.
*   `int16_t`: Signed 16-bit integer.

---

## 3. Message Definitions

### 3.1 Telemetry (STM32 $\to$ Raspberry Pi)
*ID Range: `0x10` - `0x4F`*

#### ID `0x10`: IR Sensor Array
Used for ball tracking and obstacle avoidance.
*   **Frequency:** ~20Hz
*   **Payload Length:** 32 bytes

| Offset | Type | Name | Description |
| :--- | :--- | :--- | :--- |
| 0 | `uint16_t` | ir_0 | Sensor 0 Raw Value (ADC) |
| 2 | `uint16_t` | ir_1 | Sensor 1 Raw Value |
| ... | ... | ... | ... |
| 30 | `uint16_t` | ir_15 | Sensor 15 Raw Value |

#### ID `0x11`: System Status & Yaw
Basic heading and system health.
*   **Frequency:** ~50Hz
*   **Payload Length:** 6 bytes

| Offset | Type | Name | Description |
| :--- | :--- | :--- | :--- |
| 0 | `uint32_t` | timestamp | STM32 `HAL_GetTick()` (ms since boot). Used for `dt` calculation. |
| 4 | `uint16_t` | yaw | Robot Heading. Map 0-65535 to 0-360 degrees. |

#### ID `0x12`: IMU Odometry (TBD)
High-frequency IMU data for `robot_localization` EKF.
*   **Frequency:** 50Hz-100Hz
*   **Payload Length:** TBD (Likely Quaternion + Gyro Z)

| Offset | Type | Name | Description |
| :--- | :--- | :--- | :--- |
| 0 | `float` | quat_x | BNO088 Rotation Vector X |
| 4 | `float` | quat_y | BNO088 Rotation Vector Y |
| 8 | `float` | quat_z | BNO088 Rotation Vector Z |
| 12 | `float` | quat_w | BNO088 Rotation Vector W |
| 16 | `float` | gyro_z | Angular Velocity Z (rad/s) |

#### ID `0x13`: Wheel Odometry (TBD)
Feedback from motor encoders.
*   **Payload Length:** TBD

---

### 3.2 Commands (Raspberry Pi $\to$ STM32)
*ID Range: `0x50` - `0x8F`*

#### ID `0x50`: Set Chassis Velocity (cmd_vel)
High-level holonomic control command. The STM32 handles inverse kinematics.
*   **Payload Length:** 12 bytes

| Offset | Type | Name | Description |
| :--- | :--- | :--- | :--- |
| 0 | `float` | vel_x | Linear Velocity X (Forward) in m/s |
| 4 | `float` | vel_y | Linear Velocity Y (Strafe) in m/s |
| 8 | `float` | omega | Angular Velocity (Rotation) in rad/s |

#### ID `0x51`: Set Motor RPM (Manual)
Direct low-level control of individual motors. Useful for testing or non-holonomic maneuvers.
*   **Payload Length:** 8 bytes

| Offset | Type | Name | Description |
| :--- | :--- | :--- | :--- |
| 0 | `int16_t` | m1_rpm | Motor 1 Target RPM |
| 2 | `int16_t` | m2_rpm | Motor 2 Target RPM |
| 4 | `int16_t` | m3_rpm | Motor 3 Target RPM |
| 6 | `int16_t` | m4_rpm | Motor 4 Target RPM |

#### ID `0x52`: Dribbler Control (Reserved)
Control for the ball dribbling mechanism.
*   **Payload Length:** 2 bytes
*   **Format:** TBD (Likely `uint16_t` speed or `uint8_t` state)

---

## 4. Checksum Implementation
**CRC8 Standard (Poly: `0x07`)**
*   **Init:** `0x00`
*   **XOR Out:** `0x00`
*   **Reflection:** None

The CRC is calculated over the **Raw Packet** (ID + Payload) *before* COBS encoding.

**C Reference:**
```c
uint8_t calculate_crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    while (length--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}
```