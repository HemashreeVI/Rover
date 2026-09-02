#!/usr/bin/env python3
# a_star_planner: risk-weighted A* over the live /risk_heatmap.
# Start = rover's current pose (TF). Goal = /goal_pose (RViz "2D Goal Pose").
# Publishes /planned_path (nav_msgs/Path).
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import math

from tf2_ros import Buffer, TransformListener, TransformException

RISK_WEIGHT = 2.0
OBSTACLE_THRESHOLD = 50   # heatmap value >= this = treat as blocked


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        self.risk_grid = None
        self.map_info = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/risk_heatmap', self.grid_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.get_logger().info('A* planner started. Set a goal with "2D Goal Pose" in RViz.')

    def grid_callback(self, msg):
        self.map_info = msg.info
        h, w = msg.info.height, msg.info.width
        self.risk_grid = np.array(msg.data, dtype=np.int16).reshape(h, w)

    def world_to_cell(self, x, y):
        col = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (row, col)

    def cell_to_world(self, row, col):
        x = col * self.map_info.resolution + self.map_info.origin.position.x
        y = row * self.map_info.resolution + self.map_info.origin.position.y
        return (x, y)

    def get_start_cell(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return None
        return self.world_to_cell(t.transform.translation.x, t.transform.translation.y)

    def goal_callback(self, msg):
        if self.risk_grid is None or self.map_info is None:
            self.get_logger().warning('No heatmap yet — cannot plan.')
            return
        start = self.get_start_cell()
        if start is None:
            self.get_logger().warning('No rover pose (map->base_link) — cannot plan.')
            return
        goal = self.world_to_cell(msg.pose.position.x, msg.pose.position.y)

        # --- diagnostics ---
        h, w = self.risk_grid.shape
        self.get_logger().info(f'grid={w}x{h}, start={start}, goal={goal}')
        if 0 <= start[0] < h and 0 <= start[1] < w:
            self.get_logger().info(f'start risk={self.risk_grid[start[0], start[1]]}')
        else:
            self.get_logger().info('START is OUT OF BOUNDS')
        if 0 <= goal[0] < h and 0 <= goal[1] < w:
            self.get_logger().info(f'goal risk={self.risk_grid[goal[0], goal[1]]}')
        else:
            self.get_logger().info('GOAL is OUT OF BOUNDS')

        path_cells = self.astar(start, goal)
        if path_cells is None:
            self.get_logger().warning('No path found.')
            return

        self.publish_path(path_cells)
        self.get_logger().info(f'Path planned: {len(path_cells)} cells.')

    def astar(self, start, goal):
        grid = self.risk_grid
        rows, cols = grid.shape

        def h(a, b):
            return abs(a[0]-b[0]) + abs(a[1]-b[1])

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            r, c = current
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = r+dr, c+dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                cell_risk = grid[nr, nc]
                if cell_risk >= OBSTACLE_THRESHOLD:   # blocked
                    continue
                if cell_risk < 0:                      # unknown (-1) -> treat as free
                    cell_risk = 0
                tentative_g = g_score[current] + 1 + cell_risk * RISK_WEIGHT
                neighbor = (nr, nc)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + h(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
        return None

    def publish_path(self, cells):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for (row, col) in cells:
            x, y = self.cell_to_world(row, col)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

