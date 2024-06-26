import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import time

def tahan_waktu(waktu):
    flag = True
    myTime = time.time()
    while flag == True:
        currentTime = time.time()
        if currentTime - myTime < waktu:
            flag = True
        elif currentTime - myTime >= waktu:
            flag = False

class MyNode(Node):
    def __init__(self):
        super().__init__("belok_node")
        self.subscriber_data = self.create_subscription(Int32, "/belok", self.fungsi_, 1)

        self.publish_data = self.create_publisher(Int32MultiArray, "/state", 1)
        self.talking_one = self.create_publisher(String, "/flag_belok_capit", 1)
      
    def fungsi_(self, message = Int32):
      if message.data == 11:
          info = Int32MultiArray()
          #belok kananderajat = [0, 65, 90, 180, 85, 90 ,0, 40, 110, 180, 30, 80, 5, 55, 105, 180, 80, 100, 0]
          info.data = [0, 35, 90, 180, 55, 90 ,0, 10, 100, 180, 0, 80, 5, 25, 105, 180, 50, 100, 0]
          self.publish_data.publish(info)
          tahan_waktu(3)
          self.publish_data.publish(info)
          tahan_waktu(3)
          self.publish_data.publish(info)
          tahan_waktu(3)
          self.publish_data.publish(info)
          tahan_waktu(6)
          copy = String()
          copy.data ="done"
          self.talking_one.publish(copy)


      if message.data == 22:
          info = Int32MultiArray()
          #harus 3 x cycle +- 30 derajat
          #belok kiri
          info.data = [0, 95, 90, 180, 115, 90 ,0, 70, 100, 180, 60, 80, 5, 85, 105, 180, 110,100, 0]
          self.publish_data.publish(info)
          tahan_waktu(3)
          self.publish_data.publish(info)
          tahan_waktu(3)
          self.publish_data.publish(info)
          tahan_waktu(3)
          self.publish_data.publish(info)
          tahan_waktu(6)
          copy = String()
          copy.data ="done"
          self.talking_one.publish(copy)
      



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
