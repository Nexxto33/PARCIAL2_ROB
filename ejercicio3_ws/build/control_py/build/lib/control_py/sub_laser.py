
# # base scan = sensor lidar (datos del sensor lidar) - RANGES represent distance 


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan

# class LaserSub(Node):

#     def __init__(self):
#         super().__init__("laser_sub")
#         self.sub = self.create_subscription(LaserScan, "base_scan", self.callback, 10)

#     def callback(self, msg):
#         valid_ranges = [
#             r for r in msg.ranges if r > msg.range_min and r < msg.range_max
#         ]

#         #print(msg.ranges)

#         #self.get_logger().info(f"Rangos válidos: {valid_ranges[:10]}")  #  primeros 10 valores 
        
#         # center_index = len(msg.ranges) // 2
#         # print("Distancia al frente:", msg.ranges[center_index])

#         laser_data=msg.ranges
#         print(laser_data[0])
        



# def main():
#     rclpy.init(args=None)
#     node = LaserSub()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()












#------------------ TRES DIRECCIONES ------------------
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserSub(Node):

    def __init__(self):
        super().__init__("laser_sub")
        self.sub = self.create_subscription(LaserScan, "base_scan", self.callback, 10)

    def callback(self, msg):
        laser_data = msg.ranges
        
        # Distancia a la derecha (índice 0)
        right_distance = laser_data[0]
        print(f"Distancia a la derecha: {right_distance}")
        
        # Distancia al frente (índice central)
        center_index = len(laser_data) // 2
        front_distance = laser_data[center_index]
        print(f"Distancia al frente: {front_distance}")
        
        # Distancia a la izquierda (índice final)
        left_distance = laser_data[-1]
        print(f"Distancia izquierda: {left_distance}")

def main():
    rclpy.init(args=None)
    node = LaserSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
