#!/usr/bin/env python3
import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import time
import board
import busio
import adafruit_vl53l0x


i2c = busio.I2C(board.SCL, board.SDA)
vl53_depan = adafruit_vl53l0x.VL53L0X(i2c,48)
vl53_kiri = adafruit_vl53l0x.VL53L0X(i2c,49)
vl53_kanan = adafruit_vl53l0x.VL53L0X(i2c,50)



class MyNode(Node):                      
    def __init__(self):
        super().__init__("tof1_node")
        self.talking_one = self.create_publisher(Int32MultiArray, "/tof1_", 1)
        self.subscriber_flag = self.create_subscription(String, "/flag", self.kirim_, 1)

   
    def kirim_(self, message:String):
        info = Int32MultiArray()   
        info.data = [int(vl53_depan.range/10), int(vl53_kiri.range/10), int(vl53_kanan.range/10)]
        self.talking_one.publish(info)


    
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
