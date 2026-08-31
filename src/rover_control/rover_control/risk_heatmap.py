#!/usr/bin/env python3
# ============================================================
# risk_heatmap: fuses hazards into a risk grid over the SLAM map.
# Point sensors (gas, instability) stamp at the rover.
# Camera detections (fire, smoke, human) are localized by fusing
# the bounding-box angle with LiDAR range, then stamped at the
# real world position. Multiple detections per frame supported.
# Duplicate detections of the same hazard are merged (position refined).
# ============================================================
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

from tf2_ros import Buffer, TransformListener, TransformException

# Risk scores
RISK_GAS = 8
RISK_INSTABILITY = 7
RISK_OBSTACLE = 2
RISK_FIRE = 10
RISK_SMOKE = 6
RISK_HUMAN = 3

STAMP_RADIUS = 3
RISK_CAP = 100
MERGE_RADIUS = 0.8        # meters; detections within this = same hazard

# --- calibration constants ---
CAMERA_FOV_DEG = 66.0     # standard Camera Module 3
CAMERA_LIDAR_OFFSET = 0.0 # radians; camera & LiDAR aligned (+X)

HAZARD_COLORS = {
    'fire':  (1.0, 0.0, 0.0),
    'smoke': (0.5, 0.5, 0.5),
    'gas':   (1.0, 1.0, 0.0),
    'instability': (0.6, 0.0, 0.8),
    'human': (0.0, 1.0, 0.0),
}


class RiskHeatmap(Node):
    def __init__(self):
        super().__init__('risk_heatmap')

        self.risk = None
        self.map_info = None
        self.occupancy = None
        self.scan = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Bool, '/gas_detected', self.gas_callback, 10)
        self.create_subscription(Bool, '/instability', self.instab_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Float32MultiArray, '/det/human', self.human_callback, 10)
        self.create_subscription(Float32MultiArray, '/det/fire', self.fire_callback, 10)
        self.create_subscription(Float32MultiArray, '/det/smoke', self.smoke_callback, 10)

        self.pub = self.create_publisher(OccupancyGrid, '/risk_heatmap', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/hazard_markers', 10)
        self.markers = []          # (x, y, hazard)

        self.create_timer(1.0, self.publish_heatmap)
        self.get_logger().info('Risk heatmap node started (LiDAR-camera fusion, merged markers)')

    def map_callback(self, msg):
        self.map_info = msg.info
        h, w = msg.info.height, msg.info.width
        if self.risk is None or self.risk.shape != (h, w):
            self.risk = np.zeros((h, w), dtype=np.float32)
            self.get_logger().info(f'Risk grid initialized: {w} x {h}')
        self.occupancy = np.array(msg.data, dtype=np.int8).reshape(h, w)

    def scan_callback(self, msg):
        self.scan = msg

    def get_rover_pose(self):
        if self.map_info is None:
            return None
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

    def world_to_cell(self, x, y):
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (row, col)

    def lidar_distance_at(self, angle):
        if self.scan is None:
            return None
        s = self.scan
        idx = int(round((angle - s.angle_min) / s.angle_increment))
        n = len(s.ranges)
        if idx < 0 or idx >= n:
            return None
        vals = []
        for i in range(idx - 2, idx + 3):
            if 0 <= i < n:
                r = s.ranges[i]
                if not math.isinf(r) and not math.isnan(r) and s.range_min < r < s.range_max:
                    vals.append(r)
        if not vals:
            return None
        return sum(vals) / len(vals)

    def localize_detection(self, center_x, img_width):
        pose = self.get_rover_pose()
        if pose is None:
            return None
        fov = math.radians(CAMERA_FOV_DEG)
        angle = -((center_x - img_width / 2.0) / (img_width / 2.0)) * (fov / 2.0)
        angle += CAMERA_LIDAR_OFFSET
        d = self.lidar_distance_at(angle)
        if d is None:
            return None
        x_r, y_r, yaw = pose
        px = d * math.cos(angle)
        py = d * math.sin(angle)
        mx = x_r + px * math.cos(yaw) - py * math.sin(yaw)
        my = y_r + px * math.sin(yaw) + py * math.cos(yaw)
        return (mx, my)

    def process_detections(self, data, risk_value, hazard):
        if not data or len(data) < 2:
            return
        img_width = data[0]
        for center_x in data[1:]:
            pos = self.localize_detection(center_x, img_width)
            if pos is None:
                continue
            row, col = self.world_to_cell(pos[0], pos[1])
            self._stamp_disc(row, col, risk_value)
            self._add_marker(pos[0], pos[1], hazard)

    def stamp_here(self, risk_value, hazard):
        if self.risk is None:
            return
        pose = self.get_rover_pose()
        if pose is None:
            return
        row, col = self.world_to_cell(pose[0], pose[1])
        self._stamp_disc(row, col, risk_value)
        self._add_marker(pose[0], pose[1], hazard)

    def _stamp_disc(self, row, col, risk_value):
        if self.risk is None:
            return
        h, w = self.risk.shape
        for dr in range(-STAMP_RADIUS, STAMP_RADIUS + 1):
            for dc in range(-STAMP_RADIUS, STAMP_RADIUS + 1):
                r, c = row + dr, col + dc
                if 0 <= r < h and 0 <= c < w:
                    self.risk[r, c] = min(self.risk[r, c] + risk_value, RISK_CAP)

    def _add_marker(self, x, y, hazard):
        # if a same-type marker is within MERGE_RADIUS, refine it instead of adding
        for i, (mx, my, mh) in enumerate(self.markers):
            if mh == hazard and abs(mx - x) < MERGE_RADIUS and abs(my - y) < MERGE_RADIUS:
                nx = (mx + x) / 2.0
                ny = (my + y) / 2.0
                self.markers[i] = (nx, ny, hazard)
                return
        self.markers.append((x, y, hazard))

    # callbacks
    def gas_callback(self, msg):
        if msg.data: self.stamp_here(RISK_GAS, 'gas')

    def instab_callback(self, msg):
        if msg.data: self.stamp_here(RISK_INSTABILITY, 'instability')

    def human_callback(self, msg):
        self.process_detections(list(msg.data), RISK_HUMAN, 'human')

    def fire_callback(self, msg):
        self.process_detections(list(msg.data), RISK_FIRE, 'fire')

    def smoke_callback(self, msg):
        self.process_detections(list(msg.data), RISK_SMOKE, 'smoke')

    def publish_heatmap(self):
        if self.risk is None or self.map_info is None:
            return
        combined = self.risk.copy()
        if self.occupancy is not None:
            combined[self.occupancy == 100] += RISK_OBSTACLE
        combined = np.clip(combined, 0, RISK_CAP)

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.info = self.map_info
        out.data = combined.astype(np.int8).flatten().tolist()
        self.pub.publish(out)
        self.publish_markers()

    def publish_markers(self):
        arr = MarkerArray()
        for i, (x, y, hazard) in enumerate(self.markers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'hazards'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            r, g, b = HAZARD_COLORS.get(hazard, (1.0, 1.0, 1.0))
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RiskHeatmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
