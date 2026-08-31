#!/usr/bin/env python3
# cmd_bridge: /cmd_vel -> serial letters -> ESP32 motors.
# Reads back GAS / TILT / UNSTABLE lines -> publishes ROS topics.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
import serial

PORT = '/dev/rover_esp32'
BAUD = 9600
LINEAR_THRESHOLD = 0.05
ANGULAR_THRESHOLD = 0.05


class CmdBridge(Node):
    def __init__(self):
        super().__init__('cmd_bridge')
        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        self.get_logger().info(f'Opened serial on {PORT}')

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.gas_pub = self.create_publisher(Bool, '/gas_detected', 10)
        self.instab_pub = self.create_publisher(Bool, '/instability', 10)
        self.tilt_pub = self.create_publisher(Float32MultiArray, '/tilt', 10)

        self.last_cmd = None
        self.read_timer = self.create_timer(0.05, self.read_serial)

    def cmd_callback(self, msg):
        if msg.angular.z > ANGULAR_THRESHOLD:
            cmd = 'L'
        elif msg.angular.z < -ANGULAR_THRESHOLD:
            cmd = 'R'
        elif msg.linear.x > LINEAR_THRESHOLD:
            cmd = 'F'
        elif msg.linear.x < -LINEAR_THRESHOLD:
            cmd = 'B'
        else:
            cmd = 'S'
        if cmd != self.last_cmd:
            self.ser.write(cmd.encode())
            self.get_logger().info(f'Sent: {cmd}')
            self.last_cmd = cmd

    def read_serial(self):
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode(errors='ignore').strip()

                if line.startswith('GAS:'):
                    m = Bool()
                    m.data = (line.split(':')[1] == '1')
                    self.gas_pub.publish(m)

                elif line.startswith('UNSTABLE:'):
                    m = Bool()
                    m.data = (line.split(':')[1] == '1')
                    self.instab_pub.publish(m)
                    if m.data:
                        self.get_logger().info('INSTABILITY DETECTED')

                elif line.startswith('TILT:'):
                    try:
                        parts = line.split(':')[1].split(',')
                        pitch = float(parts[0])
                        roll = float(parts[1])
                        m = Float32MultiArray()
                        m.data = [pitch, roll]
                        self.tilt_pub.publish(m)
                    except (IndexError, ValueError):
                        pass
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def destroy_node(self):
        try:
            self.ser.write(b'S')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
