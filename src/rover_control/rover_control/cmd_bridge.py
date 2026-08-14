#!/usr/bin/env python3
# ===============================
# cmd_bridge: subscribes to /cmd_vel, converts Twist -> letter,
# sends it to the ESP32 over serial. Also reads "GAS:x" from the
# ESP32 and publishes it as /gas_detected.
# ===============================
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import serial

PORT = '/dev/rover_esp32'
BAUD = 9600
# treat values smaller than this as zero (deadzone against noise/jitter)
LINEAR_THRESHOLD = 0.05
ANGULAR_THRESHOLD = 0.05


class CmdBridge(Node):
    def __init__(self):
        super().__init__('cmd_bridge')
        # open serial to the ESP32
        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        self.get_logger().info(f'Opened serial on {PORT}')

        # subscribe to /cmd_vel; cmd_callback runs on every message
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # publish gas status read back from the ESP32
        self.gas_pub = self.create_publisher(Bool, '/gas_detected', 10)

        self.last_cmd = None   # remember last letter sent (avoid spamming)

        # timer to read serial for gas data (20 Hz)
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
        # only send if the command changed (don't flood the serial port)
        if cmd != self.last_cmd:
            self.ser.write(cmd.encode())
            self.get_logger().info(f'Sent: {cmd}')
            self.last_cmd = cmd

    def read_serial(self):
        # read any lines the ESP32 sent; look for "GAS:x"
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line.startswith('GAS:'):
                    val = line.split(':')[1]
                    msg = Bool()
                    msg.data = (val == '1')
                    self.gas_pub.publish(msg)
                    if msg.data:
                        self.get_logger().info('GAS DETECTED')
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def destroy_node(self):
        # safety: stop the rover when the node shuts down
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
