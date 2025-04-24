# ----------------------
# primera si funciona 
#-----------------------


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, pow, pi

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/base_scan', self.scan_callback, 10)
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        self.front_distance = float('inf')

        self.waypoints = [(-2.2, -2.2), (-2, 2.76), (2, 2.84), (3.84, 4)]  # tus waypoints hacia el objetivo
        self.current_wp = 0
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

        # Extraer la orientación y calcular el ángulo
        orientation = msg.pose.pose.orientation
        siny = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw = atan2(siny, cosy)

    def scan_callback(self, msg):
        # Solo nos interesa el ángulo frontal
        mid_index = len(msg.ranges) // 2
        self.front_distance = msg.ranges[mid_index]

    def control_loop(self):
        if self.current_wp >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("🎉 Objetivo final alcanzado.")
            return
        
        goal_x, goal_y = self.waypoints[self.current_wp]
        dx = goal_x - self.pose_x
        dy = goal_y - self.pose_y
        distance = sqrt(dx**2 + dy**2)
        goal_angle = atan2(dy, dx)

        cmd = Twist()

        # Verificar si hay un obstáculo cercano
        if self.front_distance < 0.8:
            self.get_logger().info("🛑 Obstacle detected! Turning...")
            
            # Decidir la dirección del giro: hacia el sentido del waypoint
            if self.angle_difference(self.yaw, goal_angle) > 0:
                cmd.angular.z = 0.5  # Giro hacia la derecha
            else:
                cmd.angular.z = -0.5  # Giro hacia la izquierda
            cmd.linear.x = 0.0
        else:
            # Asegurarse de que el robot se oriente al objetivo
            if abs(self.angle_difference(self.yaw, goal_angle)) > 0.1:
                cmd.angular.z = self.angle_difference(self.yaw, goal_angle) * 0.5  # Control de orientación
                cmd.linear.x = 0.0
            else:
                # Si el robot está bien orientado y cerca del objetivo
                if distance > 0.3:
                    cmd.linear.x = 0.5
                    cmd.angular.z = 0.0  # Corrección en línea recta
                else:
                    self.get_logger().info(f"✅ Waypoint {self.current_wp+1} alcanzado.")
                    self.current_wp += 1

        self.cmd_pub.publish(cmd)

    def angle_difference(self, angle1, angle2):
        # Calcula la diferencia mínima entre dos ángulos (en radianes)
        diff = angle2 - angle1
        if diff > pi:
            diff -= 2 * pi
        elif diff < -pi:
            diff += 2 * pi
        return diff

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
