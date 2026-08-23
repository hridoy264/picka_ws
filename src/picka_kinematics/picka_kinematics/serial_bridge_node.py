#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math
import time

class RealHardwareSerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2.0)  # Wait for ESP32 reboot cycle
            self.get_logger().info(f"Connected to ESP32 on {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open port {port}: {e}")
            self.ser = None

        self.sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_state_callback,
            10
        )

    def rad_to_deg(self, rad):
        return int(round(math.degrees(rad)))

    def joint_state_callback(self, msg: JointState):
        if not self.ser or not self.ser.is_open:
            return

        joint_dict = dict(zip(msg.name, msg.position))

        # URDF Joint mapping to Degrees (Adjust offset +90 based on zero-angle mounting)
        base_deg = self.rad_to_deg(joint_dict.get('base_joint', 0.0))
        shoulder_deg = self.rad_to_deg(joint_dict.get('shoulder_joint', 0.0)) + 90
        elbow_deg = self.rad_to_deg(joint_dict.get('backarm_joint', 0.0)) + 90
        wrist_deg = self.rad_to_deg(joint_dict.get('forearm_joint', 0.0)) + 90
        gripper_deg = self.rad_to_deg(joint_dict.get('gripper_joint', 0.0))

        # Prepare packet
        packet = f"<{base_deg},{shoulder_deg},{elbow_deg},{wrist_deg},{gripper_deg}>\n"
        self.ser.write(packet.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = RealHardwareSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()