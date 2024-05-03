#!/usr/bin/env python3
import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
import time
import board
import busio
import adafruit_vl53l0x

i2c = busio.I2C(board.SCL, board.SDA)
vl53 = adafruit_vl53l0x.VL53L0X(i2c)


class MyNode(Node):                      
    def __init__(self):
        super().__init__("tof2_node")
        self.talking_one = self.create_publisher(Int32, "/tof2_", 1)
        self.subscriber_flag = self.create_subscription(String, "/flag", self.kirim_, 1)
        self.timer_ = self.create_timer(0.1, self.ngitung_tof2)
   
    def kirim_(self, message:String):
        info = Int32()   
        info.data = vl53.range/10
        self.get_logger().info("Sending ToF 2 Distance: " + str(info.data) + " cm")
        #self.talking_one.publish(info)
    
    def ngitung_tof2(self):
        self.jarak = vl53.range/10
        self.get_logger().info("Sending ToF 2 Distance: " + str(self.jarak) + " cm")

    
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
