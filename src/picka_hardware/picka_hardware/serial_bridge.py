import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory

try:
    import serial
except ImportError:
    serial = None


ARM_JOINTS = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_joint']


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('command_topic', '/arm_controller/joint_trajectory')
        self.declare_parameter('gripper_topic', '/gripper/command')
        self.declare_parameter('reconnect_seconds', 2.0)
        self._serial = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._gripper_deg = 0.0

        command_topic = self.get_parameter('command_topic').value
        gripper_topic = self.get_parameter('gripper_topic').value
        self.create_subscription(JointTrajectory, command_topic, self._trajectory, 10)
        self.create_subscription(Float64, gripper_topic, self._gripper, 10)
        self._state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self._thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

    def _connect(self):
        if serial is None:
            self.get_logger().error('pyserial is missing: sudo apt install python3-serial')
            return False
        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1, write_timeout=0.5)
            time.sleep(2.0)  # ESP32 commonly resets when USB serial opens.
            self._serial.reset_input_buffer()
            self.get_logger().info(f'Connected to ESP32 on {port} at {baud} baud')
            return True
        except (OSError, serial.SerialException) as error:
            self.get_logger().warning(f'Cannot open {port}: {error}')
            self._serial = None
            return False

    def _send(self, line):
        with self._lock:
            if self._serial is None:
                self.get_logger().warning('ESP32 is not connected; command discarded')
                return
            try:
                self._serial.write((line + '\n').encode('ascii'))
            except (OSError, serial.SerialException) as error:
                self.get_logger().error(f'ESP32 write failed: {error}')
                self._serial.close()
                self._serial = None

    def _trajectory(self, message):
        if not message.points:
            self.get_logger().warning('Received an empty trajectory')
            return
        point = message.points[-1]
        if len(point.positions) != len(message.joint_names):
            self.get_logger().error('Trajectory joint names and positions do not match')
            return
        positions = dict(zip(message.joint_names, point.positions))
        missing = [name for name in ARM_JOINTS if name not in positions]
        if missing:
            self.get_logger().error(f'Trajectory is missing joints: {missing}')
            return
        degrees = [math.degrees(float(positions[name])) for name in ARM_JOINTS]
        duration_ms = max(100, point.time_from_start.sec * 1000 + point.time_from_start.nanosec // 1_000_000)
        self._send('MOVE,' + ','.join(f'{value:.3f}' for value in degrees) + f',{self._gripper_deg:.3f},{duration_ms}')

    def _gripper(self, message):
        self._gripper_deg = max(0.0, min(90.0, float(message.data)))
        self._send(f'GRIP,{self._gripper_deg:.3f}')

    def _handle_line(self, line):
        fields = line.strip().split(',')
        if len(fields) == 6 and fields[0] == 'STATE':
            try:
                degrees = [float(value) for value in fields[1:]]
            except ValueError:
                return
            state = JointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.name = ARM_JOINTS + ['gripper_joint']
            state.position = [math.radians(value) for value in degrees]
            self._state_pub.publish(state)
        elif line.startswith('ERROR,'):
            self.get_logger().error(f'ESP32: {line[6:]}')

    def _serial_loop(self):
        delay = float(self.get_parameter('reconnect_seconds').value)
        while not self._stop.is_set():
            if self._serial is None:
                if not self._connect():
                    self._stop.wait(delay)
                continue
            try:
                data = self._serial.readline()
                if data:
                    self._handle_line(data.decode('ascii', errors='replace'))
            except (OSError, serial.SerialException) as error:
                self.get_logger().error(f'ESP32 connection lost: {error}')
                self._serial.close()
                self._serial = None

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._serial is not None:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
