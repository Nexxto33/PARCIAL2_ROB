

#------------------------------------------------------
#   PLANEAR NAVEGACION USANDO A* MEJROADO - FUNCIONA
#------------------------------------------------------


import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, pi


class AStarNavigator(Node):

    origin = (-8.0, -8.0)
    resolution = 0.25

    def __init__(self):
        super().__init__('astar_navigator')
        self.get_logger().info(' AStar Navigator iniciado')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'ground_truth', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'base_scan', self.scan_callback, 10)

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        self.grid_map = np.zeros((64, 64), dtype=np.uint8)

        self.goal_world = (4.0, 4.0)
        self.goal_grid = self.world_to_grid(*self.goal_world)
        self.path = []
        self.current_wp = 0

        self.timer = self.create_timer(0.1, self.control_loop)

    def reached_goal(self, tol=0.3):
        dx = self.pose_x - self.goal_world[0]
        dy = self.pose_y - self.goal_world[1]
        return sqrt(dx**2 + dy**2) < tol

    @staticmethod
    def world_to_grid(x, y):
        gx = int((x - AStarNavigator.origin[0]) / AStarNavigator.resolution)
        gy = int((y - AStarNavigator.origin[1]) / AStarNavigator.resolution)
        return gx, gy

    @staticmethod
    def grid_to_world(gx, gy):
        x = gx * AStarNavigator.resolution + AStarNavigator.origin[0]
        y = gy * AStarNavigator.resolution + AStarNavigator.origin[1]
        return x, y

    @staticmethod
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def astar(start, goal, grid):
        if grid[start[0], start[1]] == 1:
            found = False
            for radius in range(1, 4):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        nx, ny = start[0] + dx, start[1] + dy
                        if (0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]
                                and grid[nx, ny] == 0):
                            start = (nx, ny)
                            found = True
                            break
                    if found:
                        break

        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while open_list:
            _, current = heapq.heappop(open_list)
            if current == goal:
                break

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (1, 1), (-1, 1), (1, -1)]:
                next_node = (current[0] + dx, current[1] + dy)
                if (0 <= next_node[0] < grid.shape[0] and
                    0 <= next_node[1] < grid.shape[1] and
                    grid[next_node] == 0):

                    new_cost = cost_so_far[current] + 1
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + AStarNavigator.heuristic(goal, next_node)
                        heapq.heappush(open_list, (priority, next_node))
                        came_from[next_node] = current

        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                return []
        path.append(start)
        path.reverse()
        return path

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2 * (q.w * q.z + q.x * q.y)
        cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = atan2(siny, cosy)

    def scan_callback(self, msg):
        self.grid_map.fill(0)
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        n = len(ranges)

        front = ranges[n//2 - 10:n//2 + 10]
        left = ranges[int(n*0.75):int(n*0.95)]
        right = ranges[int(n*0.05):int(n*0.25)]

        def safe_min(arr):
            valid = [r for r in arr if 0.2 < r < 5.0]
            return min(valid) if valid else float('inf')

        self.front_distance = safe_min(front)
        self.left_distance = safe_min(left)
        self.right_distance = safe_min(right)

        for i, r in enumerate(ranges):
            if 0.2 < r < 5.0:
                angle = angle_min + i * angle_increment
                x = self.pose_x + r * np.cos(self.yaw + angle)
                y = self.pose_y + r * np.sin(self.yaw + angle)
                gx, gy = self.world_to_grid(x, y)
                if 0 <= gx < self.grid_map.shape[0] and 0 <= gy < self.grid_map.shape[1]:
                    self.grid_map[gx, gy] = 1

        kernel = 3
        inflated = self.grid_map.copy()
        for x in range(self.grid_map.shape[0]):
            for y in range(self.grid_map.shape[1]):
                if self.grid_map[x, y] == 1:
                    for dx in range(-kernel, kernel + 1):
                        for dy in range(-kernel, kernel + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.grid_map.shape[0] and 0 <= ny < self.grid_map.shape[1]:
                                inflated[nx, ny] = 1
        self.grid_map = inflated

    def control_loop(self):
        if self.reached_goal():
            self.stop_robot()
            self.get_logger().info("✅ Posición final alcanzada (por ground_truth).")
            self.path = []
            return

        if not self.path:
            current_grid = self.world_to_grid(self.pose_x, self.pose_y)
            self.path = self.astar(current_grid, self.goal_grid, self.grid_map)
            self.current_wp = 0
            if not self.path:
                self.get_logger().warn("⚠️ No se encontró ruta al objetivo.")
                return

        if self.current_wp >= len(self.path):
            self.stop_robot()
            self.get_logger().info("🎯 Objetivo final alcanzado.")
            return

        goal_x, goal_y = self.grid_to_world(*self.path[self.current_wp])
        dx = goal_x - self.pose_x
        dy = goal_y - self.pose_y
        distance = sqrt(dx ** 2 + dy ** 2)
        goal_angle = atan2(dy, dx)
        angle_diff = self.angle_difference(self.yaw, goal_angle)

        cmd = Twist()

        if self.front_distance < 0.8 and distance > 0.3:
            self.get_logger().warn("🚧 Camino bloqueado al frente. Recalculando ruta...")
            current_pos = self.world_to_grid(self.pose_x, self.pose_y)
            self.path = self.astar(current_pos, self.goal_grid, self.grid_map)
            self.current_wp = 0
            return

        elif self.front_distance < 0.9:
            self.get_logger().warn(f"🛑 Obstáculo frente a {self.front_distance:.2f}m")
            if self.left_distance > self.right_distance:
                self.get_logger().info("↩️ Evadiendo hacia la izquierda")
                cmd.angular.z = 0.8
            else:
                self.get_logger().info("↪️ Evadiendo hacia la derecha")
                cmd.angular.z = -0.8
            cmd.linear.x = 0.3

        else:
            if abs(angle_diff) > 0.2:
                cmd.angular.z = angle_diff * 0.5
            elif distance > 0.3:
                cmd.linear.x = 0.5
            else:
                self.get_logger().info(f"✅ Waypoint {self.current_wp+1}/{len(self.path)} alcanzado.")
                self.current_wp += 1

        self.cmd_pub.publish(cmd)

    def angle_difference(self, a1, a2):
        diff = a2 - a1
        if diff > pi:
            diff -= 2 * pi
        elif diff < -pi:
            diff += 2 * pi
        return diff

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
