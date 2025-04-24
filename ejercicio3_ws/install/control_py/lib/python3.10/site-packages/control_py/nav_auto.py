
#---------------------------------
#   PLANEAR NAVEGACION USANDO A*
#---------------------------------



import heapq
import numpy as np
import rclpy
from rclpy.node import Node


class AStarNavigator(Node):
    def __init__(self):
        super().__init__('astar_navigator')
        self.get_logger().info('AStar Navigator iniciado')

    # =============================
    # Configuración del entorno
    # =============================
    resolution = 0.25  # metros por celda
    origin = (-8.0, -8.0)  # esquina inferior izquierda del mapa
    map_width = int(16.0 / resolution)  # 64 celdas
    map_height = int(16.0 / resolution)  # 64 celdas

    # =============================
    # Mapeo entre coordenadas
    # =============================
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

    # =============================
    # A* Planning
    # =============================
    @staticmethod
    def astar(start, goal, grid):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                break

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                next_node = (current[0] + dx, current[1] + dy)

                if (0 <= next_node[0] < grid.shape[0] and
                        0 <= next_node[1] < grid.shape[1] and
                        grid[next_node] == 0):  # sin obstáculo

                    new_cost = cost_so_far[current] + 1
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + AStarNavigator.heuristic(goal, next_node)
                        heapq.heappush(open_list, (priority, next_node))
                        came_from[next_node] = current

        # reconstruir camino
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                return []  # sin camino
        path.append(start)
        path.reverse()
        return path

    @staticmethod
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # distancia de Manhattan


def main(args=None):
    rclpy.init(args=args)
    
    # Crear la instancia del nodo AStarNavigator
    node = AStarNavigator()

    # =============================
    # Mapa de obstáculos
    # =============================
    # Para este ejemplo, mapa vacío (sin obstáculos)
    grid_map = np.zeros((AStarNavigator.map_width, AStarNavigator.map_height), dtype=np.uint8)

    # Posición inicial y final del robot según .world
    start_world = (-7.0, -7.0)
    goal_world = (5.0, 4.0)

    start_grid = AStarNavigator.world_to_grid(*start_world)
    goal_grid = AStarNavigator.world_to_grid(*goal_world)

    # =============================
    # Planificación de ruta
    # =============================
    path = AStarNavigator.astar(start_grid, goal_grid, grid_map)

    # =============================
    # Mostrar camino encontrado
    # =============================
    node.get_logger().info("📍 Camino A*:")
    for gx, gy in path:
        wx, wy = AStarNavigator.grid_to_world(gx, gy)
        node.get_logger().info(f" -> ({wx:.2f}, {wy:.2f})")

    # Mantener el nodo en ejecución hasta que se interrumpa
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
