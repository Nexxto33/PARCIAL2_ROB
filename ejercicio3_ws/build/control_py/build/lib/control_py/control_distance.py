import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Control(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.sub = self.create_subscription(LaserScan, "base_scan", self.scan_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.obstacle_detected = False

    def scan_callback(self, msg):
        # Toma la distancia al frente (ángulo 0°)
        mid_index = len(msg.ranges) // 2
        distance = msg.ranges[mid_index]

        # Verifica si hay obstáculo a menos de 1 m
        if distance < 1.0:
            self.obstacle_detected = True
            self.get_logger().info(f"🛑 Obstáculo detectado a {distance:.2f} m. Deteniendo.")
        else:
            self.obstacle_detected = False

    def timer_callback(self):
        msg = Twist()
        if not self.obstacle_detected:
            msg.linear.x = 0.5  # velocidad constante
        else:
            msg.linear.x = 0.0  # detenerse
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
