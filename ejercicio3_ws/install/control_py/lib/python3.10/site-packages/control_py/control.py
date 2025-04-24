import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Control(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        # msg.angular.z = - 0.1
        self.pub.publish(msg)

def main():
    rclpy.init(args=None)
    node = Control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
