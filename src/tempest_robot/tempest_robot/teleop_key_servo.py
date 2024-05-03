import rclpy
from rclpy.node import Node
from getkey import getkey, keys
from std_msgs.msg import Int32MultiArray
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

GPIO.BUTTON = 21

GPIO.setup(GPIO.BUTTON, GPIO.IN)
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
        super().__init__("teleop_key_servo")
        self.talking_one = self.create_publisher(Int32MultiArray, "/state", 1)
        self.check_capit = False
        self.timer_ = self.create_timer(0.1, self.send_message)
        self.get_logger().info("W (Maju) | A (Menghadap Kiri) | S (Capit) | D (Menghadap Kanan)| X (Mundur)")
    def send_message(self):
        # if GPIO.input(GPIO.BUTTON)==1:
        #     self.get_logger().info("Aktif")
        key = getkey()
        message = Int32MultiArray()
        if key == 'w':
            self.get_logger().info("Maju") #yang diubah yang index 1 sama 4 sama 7, - 20 -20 +20, -20 +20 +20
            message.data = [0, 60, 90, 140, 45, 95 ,15, 60, 110, 180, 10, 80, 5, 85, 105, 170, 85, 95, 25]
            self.talking_one.publish(message)
        elif key == 'x':
            self.get_logger().info("Mundur") # yang diubah yang index 1 sama 4 sama 7
            message.data = [0, 100, 90, 140, 85, 95 ,15, 20, 110, 180, 50, 80, 5, 45, 105, 170, 45, 95, 25]
            self.talking_one.publish(message)

        elif key == 'd':
            self.get_logger().info("Menghadap Kanan")
            message.data = [0, 60, 90, 140, 45, 95 ,15, 20, 110, 180, 10, 80, 5, 45, 105, 170, 45, 95, 25]
            self.talking_one.publish(message)

        elif key == 'a':
            self.get_logger().info("Menghadap Kiri")
            message.data = [0, 100, 90, 140, 85, 95 ,15, 60, 110, 180, 50, 80, 5, 85, 105, 170, 85, 95, 25]
            self.talking_one.publish(message)

        elif key == ' ':
            self.get_logger().info("Berdiri")
            message.data = [1, 80, 90, 140, 65, 95 ,15, 40, 110, 180, 30, 80, 5, 65, 105, 170, 65, 95, 25]
            self.talking_one.publish(message)
        elif key == 's':
            if self.check_capit == False:
                #turun
                self.get_logger().info("Turun")         
                message.data = [2, 80, 110, 140, 65, 95 ,15, 40, 130, 180, 30, 80, 5, 65, 105, 170, 65, 75, 25]
                self.talking_one.publish(message)
                tahan_waktu(3)
                #maju
                self.get_logger().info("Maju")
                message.data = [3, 60, 110, 140, 45, 95 ,15, 60, 110, 180, 10, 80, 5, 85, 105, 170, 85, 75, 25]
                self.talking_one.publish(message)
                tahan_waktu(3)
                #capit
                self.get_logger().info("Capit")
                message.data = [4, 80, 110, 140, 65, 95 ,15, 40, 130, 180, 30, 80, 5, 65, 105, 170, 65, 75, 25]
                self.talking_one.publish(message)
                tahan_waktu(3)
                #mundur
                self.get_logger().info("Mundur")
                message.data = [0, 100, 90, 140, 85, 95 ,15, 20, 110, 180, 50, 80, 5, 45, 105, 170, 45, 95, 25]
                self.talking_one.publish(message)
                self.check_capit = True
                # tahan_waktu(3)
                # #berdiri
                # self.get_logger().info("Berdiri")
                # message.data = [1, 80, 90, 140, 65, 95 ,15, 40, 110, 180, 30, 80, 5, 65, 105, 170, 65, 95, 25]
                # self.talking_one.publish(message)
            elif self.check_capit == True:
                #capit
                self.get_logger().info("Capit")
                message.data = [4, 80, 110, 140, 65, 95 ,15, 40, 130, 180, 30, 80, 5, 65, 105, 170, 65, 75, 25]
                self.talking_one.publish(message)
                tahan_waktu(3)
                self.check_capit = False
        elif key == 'c':
            self.get_logger().info("Geser kanan")
            message.data = [5, 80, 90, 140, 65, 95 ,15, 40, 110, 180, 30, 80, 5, 65, 105, 170, 65, 95, 25]
            self.talking_one.publish(message)
        elif key == 'z':
            self.get_logger().info("Geser kiri")
            message.data = [6, 80, 90, 140, 65, 95 ,15, 40, 110, 180, 30, 80, 5, 65, 105, 170, 65, 95, 25]
            self.talking_one.publish(message)
        elif key == 'z':
            self.get_logger().info("Berdiri")
            message.data = [6, 80, 90, 140, 65, 95 ,15, 40, 110, 180, 30, 80, 5, 65, 105, 170, 65, 95, 25]
            self.talking_one.publish(message)
        else:
            self.get_logger().info("Wrong Key")
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
