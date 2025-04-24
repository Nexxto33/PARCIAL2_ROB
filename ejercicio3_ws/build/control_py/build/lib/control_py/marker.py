#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ObjetivoMarkerNode(Node):
    def __init__(self):
        super().__init__('objetivo_marker')
        self.publisher = self.create_publisher(Marker, '/objetivo_marker', 10)
        self.timer = self.create_timer(0.5, self.publicar_marker)
        self.get_logger().info("📍 Publicando marcador de objetivo en (5.0, 4.0)")

    def publicar_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"  # ¡Usa el mismo frame que tu robot!
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "objetivo"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 5.0
        marker.pose.position.y = 4.0
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ObjetivoMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
