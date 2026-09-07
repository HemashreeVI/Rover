#!/usr/bin/env python3
# path_follower: drives the rover along /planned_path using /cmd_vel.
# Turn-then-drive control, suited to skid-steer + discrete cmd_bridge.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import math

from tf2_ros import Buffer, TransformListener, TransformException

# tuning
WAYPOINT_TOLERANCE = 0.20   # m: consider a waypoint reached within this
GOAL_TOLERANCE = 0.15       # m: stop when this close to final goal
ANGLE_TOLERANCE = 0.35      # rad (~20 deg): if heading error under this, drive forward
LINEAR_SPEED = 0.2          # m/s command when driving forward
ANGULAR_SPEED = 0.5         # rad/s command when turning


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.path = []          # list of (x, y) waypoints
        self.current_wp = 0     # index of the waypoint we're heading to

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # control loop at 5 Hz
        self.create_timer(0.2, self.control_loop)

        self.get_logger().info('Path follower started. Waiting for a path...')

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_wp = 0
        self.get_logger().info(f'New path received: {len(self.path)} waypoints.')

    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    def stop(self):
        self.cmd_pub.publish(Twist())   # all zeros = stop

    def control_loop(self):
        if not self.path or self.current_wp >= len(self.path):
            return

        pose = self.get_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        # target waypoint
        tx, ty = self.path[self.current_wp]
        dx, dy = tx - rx, ty - ry
        dist = math.hypot(dx, dy)

        # reached final goal?
        if self.current_wp == len(self.path) - 1 and dist < GOAL_TOLERANCE:
            self.stop()
            self.get_logger().info('Goal reached.')
            self.path = []          # stop following
            return

        # reached this waypoint? advance to next
        if dist < WAYPOINT_TOLERANCE:
            self.current_wp += 1
            return

        # angle to the target vs current heading
        target_angle = math.atan2(dy, dx)
        angle_err = self.normalize(target_angle - ryaw)

        cmd = Twist()
        if abs(angle_err) > ANGLE_TOLERANCE:
            # turn toward target
            cmd.angular.z = ANGULAR_SPEED if angle_err > 0 else -ANGULAR_SPEED
        else:
            # facing it — drive forward
            cmd.linear.x = LINEAR_SPEED
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize(angle):
        # wrap to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
