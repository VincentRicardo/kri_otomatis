import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import math

[0, 65, 90, 140, 85, 105 ,25, 40, 110, 180, 30, 80, 5, 55, 85, 150, 80, 85, 15]
class MyNode(Node):
    def __init__(self):
        super().__init__("olah_node")
        self.subscriber_data = self.create_subscription(Int32MultiArray, "/belok", self.fungsi_, 1)

        self.publish_data = self.create_publisher(Int32MultiArray, "/state", 1)
      
    def fungsi_(self, message = Int32MultiArray):
      info = Int32MultiArray()
      #harus 3 x cycle +- 30 derajat
      #belok kiri
      info.data = [0, 95, 90, 140, 115, 105 ,25, 70, 110, 180, 60, 80, 5, 85, 85, 150, 110, 85, 15]
      #belok kanan
      info.data = [0, 35, 90, 140, 55, 105 ,25, 10, 110, 180, 0, 80, 5, 25, 85, 150, 50, 85, 15]
      self.publish_data.publish(info)



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
