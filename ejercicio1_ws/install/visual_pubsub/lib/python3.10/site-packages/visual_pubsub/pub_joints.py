# src/visual_pubsub/pub_joints.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class CustomJointPublisher(Node):
    def __init__(self):
        super().__init__('cinematica_directa')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        
        msg.name = ['q1', 'q2', 'q3', 'q4']
        q1 = 0.5
        q2 = -0.3
        q3 = 0.2
        q4 = 0.1   # dentro del rango
        msg.position = [q1, q2, q3, q4]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Joint States: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = CustomJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
