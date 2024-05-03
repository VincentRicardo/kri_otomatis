#!/usr/bin/env python3
import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import time
import math
from DFRobot_BMX160 import BMX160

bmx = BMX160(1)
while not bmx.begin():
    pass


class MyNode(Node):                      
    def __init__(self):
        super().__init__("gyro_node")
        self.talking_one = self.create_publisher(Int32MultiArray, "/gyro_", 1)
        self.subscriber_flag = self.create_subscription(String, "/flag", self.kirim_, 1)
        self.timer_ = self.create_timer(0.1, self.ngitung_gyro)
        self.measurement = 0
        self.yaw = 0
        self.pitch = 0
        self.previousTime = time.time()
        self.gyroAngleX = 0
        self.gyroAngleY = 0
   
    def kirim_(self, message:String):
        info = Int32MultiArray()   
        info.data = [int(self.yaw), int(self.pitch)]
        self.get_logger().info("Sending Gyro Yaw = " + str(info.data[0]) + ", Sending Gyro Pitch = " + str(info.data[1]))
        self.talking_one.publish(info)
        

    def ngitung_gyro(self):
        self.currentTime = time.time()
        self.elapsedTime = self.currentTime - self.previousTime
        data= bmx.get_all_data()
        GyroX = data[3]
        GyroY = data[4]
        GyroZ = data[5]

        GyroX = GyroX + 0.04108258 # cek imu error
        GyroY = GyroY + 0.02107483 # cek imu error
        GyroZ = GyroZ + 0.03761457 # cek imu error
        AccelX = data[6]
        AccelY = data[7]
        AccelZ = data[8]
        AccelAngleY = ((math.atan(-1 * (AccelX)/math.sqrt(math.pow((AccelY),2) + math.pow((AccelZ),2)))*180/math.pi)) - 3.2131138563935764 # cek imu error
        
        self.gyroAngleX = self.gyroAngleX + GyroX * self.elapsedTime
        self.gyroAngleY = self.gyroAngleY + GyroY * self.elapsedTime
        self.yaw =  self.yaw + (GyroZ * self.elapsedTime *8.18182)

        self.pitch = (0.96 * self.gyroAngleY + 0.04 * AccelAngleY) * 12.85714

        print("Pitch: " + str(self.pitch))
        print("Yaw: " + str(self.yaw))
        print(" ")
        self.previousTime = self.currentTime

    
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
