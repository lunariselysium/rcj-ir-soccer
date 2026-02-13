#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
import threading
import time
from cobs import cobs

# ROS Messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16MultiArray, Float32, String

# ==========================================
# CONFIGURATION CONSTANTS
# ==========================================
# Protocol IDs
ID_IR_ARRAY     = 0x10
ID_SYS_STATUS   = 0x11
ID_IMU          = 0x12
ID_DEBUG_MSG    = 0x4F
ID_CMD_VEL      = 0x50

class STM32SerialBridge(Node):
    def __init__(self):
        super().__init__('stm32_serial_bridge')

        # 1. Declare Parameters (Can be set via launch file)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # 2. Initialize Serial Port
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to STM32 on {port} @ {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            exit(1)

        # 3. Setup Publishers (STM32 -> ROS)
        # Using UInt16MultiArray for the 16 IR sensors
        self.pub_ir = self.create_publisher(UInt16MultiArray, 'sensors/ir_array', 10)
        # Using Float32 for Yaw (or you could use Odometry msg)
        self.pub_yaw = self.create_publisher(Float32, 'robot/yaw', 10)
        # Standard IMU message
        self.pub_imu = self.create_publisher(Imu, 'sensors/imu', 10)
        # Debug string
        self.pub_debug = self.create_publisher(String, 'stm32/debug', 10)

        # 4. Setup Subscribers (ROS -> STM32)
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 5. Start Serial Read Thread
        # We use a separate thread so serial reading doesn't block ROS callbacks
        self.rx_buffer = bytearray()
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    # ==========================================
    # HELPER: CRC8
    # ==========================================
    def calculate_crc8(self, data):
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    # ==========================================
    # ROS -> STM32 (Callbacks)
    # ==========================================
    def cmd_vel_callback(self, msg: Twist):
        """
        Translates geometry_msgs/Twist to ID 0x50
        Payload: [vel_x(float), vel_y(float), omega(float)]
        """
        try:
            # Extract data (ROS uses meters/sec and rad/sec, same as your protocol)
            vx = float(msg.linear.x)
            vy = float(msg.linear.y)
            wz = float(msg.angular.z)

            # Pack Payload: Little Endian (<), 3 floats (f)
            payload = struct.pack('<fff', vx, vy, wz)
            
            self.send_packet(ID_CMD_VEL, payload)
        except Exception as e:
            self.get_logger().warn(f"Failed to process cmd_vel: {e}")

    def send_packet(self, packet_id, payload):
        """Encodes and writes packet to serial"""
        raw_packet = bytes([packet_id]) + payload
        crc_val = self.calculate_crc8(raw_packet)
        packet_with_crc = raw_packet + bytes([crc_val])
        
        try:
            encoded = cobs.encode(packet_with_crc)
            self.ser.write(encoded + b'\x00')
        except Exception as e:
            self.get_logger().error(f"Serial Write Error: {e}")

    # ==========================================
    # STM32 -> ROS (Read Loop)
    # ==========================================
    def serial_read_loop(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting)
                    for byte in chunk:
                        if byte == 0x00:
                            # End of packet found
                            if len(self.rx_buffer) > 0:
                                self.decode_and_publish(bytes(self.rx_buffer))
                                self.rx_buffer = bytearray()
                        else:
                            self.rx_buffer.append(byte)
                else:
                    time.sleep(0.001) # Prevent CPU hogging
            except Exception as e:
                self.get_logger().error(f"Serial Read Error: {e}")
                time.sleep(1)

    def decode_and_publish(self, raw_data):
        try:
            decoded = cobs.decode(raw_data)
        except cobs.DecodeError:
            self.get_logger().warn("COBS Decode Error")
            return

        if len(decoded) < 2: return # Too short

        packet_id = decoded[0]
        payload = decoded[1:-1]
        received_crc = decoded[-1]

        # Verify CRC
        if self.calculate_crc8(decoded[:-1]) != received_crc:
            self.get_logger().warn("CRC Checksum Mismatch")
            return

        # Route Packet
        if packet_id == ID_IR_ARRAY:
            self.handle_ir_data(payload)
        elif packet_id == ID_SYS_STATUS:
            self.handle_status(payload)
        elif packet_id == ID_IMU:
            self.handle_imu(payload)
        elif packet_id == ID_DEBUG_MSG:
            self.handle_debug(payload)

    # ==========================================
    # PAYLOAD HANDLERS
    # ==========================================
    def handle_ir_data(self, payload):
        # 16 sensors * 2 bytes = 32 bytes
        if len(payload) != 32: return
        
        # Unpack 16 unsigned shorts
        values = struct.unpack('<16H', payload)
        
        msg = UInt16MultiArray()
        msg.data = list(values)
        self.pub_ir.publish(msg)

    def handle_status(self, payload):
        # Timestamp (uint32) + Yaw (uint16) = 6 bytes
        if len(payload) != 6: return
        
        ts, raw_yaw = struct.unpack('<IH', payload)
        
        # Map 0-255 to 0-360 degrees
        yaw_deg = (raw_yaw / 255.0) * 360.0
        
        msg = Float32()
        msg.data = yaw_deg
        self.pub_yaw.publish(msg)

    def handle_imu(self, payload):
        # 4 floats (quat) + 1 float (gyro z) = 20 bytes
        if len(payload) != 20: return
        
        qx, qy, qz, qw, gyro_z = struct.unpack('<fffff', payload)
        
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.z = gyro_z
        
        # Note: You might need to populate covariance matrices here 
        # for robot_localization to work well.
        
        self.pub_imu.publish(msg)

    def handle_debug(self, payload):
        try:
            text = payload.decode('utf-8', errors='ignore')
            msg = String()
            msg.data = text
            self.pub_debug.publish(msg)
            # Also print to console
            print(f"STM32: {text}", flush=True)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = STM32SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
