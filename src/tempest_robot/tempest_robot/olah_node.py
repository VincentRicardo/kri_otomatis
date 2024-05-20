import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import math

from getkey import getkey, keys
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.BUTTON = 21
GPIO.LED = 16

GPIO.setup(GPIO.BUTTON, GPIO.IN)
GPIO.setup(GPIO.LED, GPIO.OUT)
GPIO.output(GPIO.LED, GPIO.LOW)



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
        super().__init__("olah_node")
        self.gyro_flag = False
        self.cam_flag = False
        self.first_flag = False

        self.capit_flag = False #flag buat dalam kondisi lagi nyapi
        self.lepas = True #sebaliknya

        self.flag = 10
        self.flag_belok_capit = False #flag lagi proses belok atau nyapit

        self.maju = False
        self.kiri = True
        self.kanan = False

        self.tof1_flag = False
        self.tof2_flag = False
        self.tof3_flag = False

        self.mode1 = True
        self.mode2 = False
        self.mode3 = False
        self.mode4 = False
        self.mode5 = False

        self.subscriber_gyro = self.create_subscription(Int32MultiArray, "/gyro_", self.gyro_, 1)
        self.subscriber_cam = self.create_subscription(Int32MultiArray, "/cam_", self.cam_, 1)
        self.subscriber_flag_belok_capit = self.create_subscription(String, "/flag_belok_capit", self.flag_function, 1)
        self.subscriber_tof1 = self.create_subscription(Int32MultiArray, "/tof1_", self.tof1_, 1)
        self.subscriber_tof2 = self.create_subscription(Int32, "/tof2_", self.tof2_, 1)
        self.subscriber_tof3 = self.create_subscription(Int32, "/tof3_", self.tof3_, 1)


        self.publish_data = self.create_publisher(Int32MultiArray, "/state", 1)
        self.maju_mundur_data = self.create_publisher(Int32, "/maju_mundur", 1)
        self.belok_data = self.create_publisher(Int32, "/belok", 1)
        self.capit_data = self.create_publisher(Int32MultiArray, "/capit", 1)

        self.timer_ = self.create_timer(0.1, self.send_mode_call)
        self.first_ = self.create_timer(5, self.send_first)

        self.cam = [0,0]
        self.gyro = [0,0]
        self.tof1 = 0
        self.tof2 = 0
        self.tof3 = 0
        

    
    def cam_(self, message = Int32MultiArray):
        self.get_logger().info("Receiving Cam Distance = " + str(message.data[0]) + " cm & Angle = " + str(message.data[1]))
        
        self.cam = [message.data[0], message.data[1]]

        self.cam_flag = True

    def gyro_(self, message = Int32MultiArray):
        self.get_logger().info("Receiving Yaw = " + str(message.data[0])+ " & Pitch = " + str(message.data[1]))
        self.gyro = [message.data[0], message.data[1]]
        self.gyro_flag = True

    #depan robot
    def tof1_ (self, message = Int32):
        self.get_logger().info("Receiving ToF 1 Distance: " + str(message.data) + " cm")
        self.tof1 = message.data[0]
        self.tof2 = message.data[1]
        self.tof3 = message.data[2]
        self.tof1_flag = True
        self.tof2_flag = True
        self.tof3_flag = True
        
    #sebelah kiri robot
    def tof2_ (self, message = Int32):
        self.get_logger().info("Receiving ToF 2 Distance: " + str(message.data) + " cm")
        self.tof2 = message.data
        self.tof2_flag = True
        
    #sebelah kanan robot
    def tof3_ (self, message = Int32):
        self.get_logger().info("Receiving ToF 3 Distance: " + str(message.data) + " cm")
        self.tof3 = message.data
        self.tof3_flag = True
    
    def flag_function(self, message:String):
        self.flag_belok_capit = False

    def send_first(self):
        if self.first_flag == False:
            ddd = Int32MultiArray()
            ddd.data = [1, 65, 90, 140, 85, 90 ,15, 40, 110, 180, 30, 80, 5, 55, 105, 180, 80, 90, 20]
            self.publish_data.publish(ddd)
            self.first_flag = True


    def send_mode_call(self):
        if self.gyro_flag == True and self.tof1_flag == True and self.tof2_flag == True and self.tof3_flag == True and self.flag_belok_capit == False:
            if self.mode1 == True:
                if self.tof1 <= 20 and self.tof2 <= 30 and self.flag == 10:
                    #belok kanan 90 derajat

                    belok = Int32()
                    belok.data = 11
                    self.belok_data.publish(belok)
                    self.flag_belok_capit = True
                    self.get_logger().info("Ngadep Kanan")
                    self.flag = 0
                    
                elif self.tof1 <= 20 and self.tof3 <= 30 and self.flag == 10:
                    #belok kiri 90 derajat
                    
                    belok = Int32()
                    belok.data = 22
                    self.belok_data.publish(belok)
                    self.flag_belok_capit = True
                    self.get_logger().info("Ngadep Kiri")
                    self.flag = 0
                
                elif self.capit_flag == True and self.lepas == True and self.flag == 0:
                    # #nyapit
                    # data = Int32MultiArray()
                    # data.data = [int(self.cam[0]), int(self.cam[1])] #jarak, angle
                    # self.capit_data.publish(data)
                    # self.flag_belok_capit = True
                    self.lepas == False
                    self.capit_flag = False
                    self.get_logger().info("Nyapit")
                elif self.flag == 2:
                    # data = Int32MultiArray()
                    # data.data = [int(self.cam[0]), int(self.cam[1])] #jarak, angle
                    # self.capit_data.publish(data)
                    # self.flag_belok_capit = True
                    self.lepas == True
                    self.get_logger().info("Lepas Nyapit")
                    self.flag = 0
                    self.kanan = False
                    self.kiri = True
                    self.mode1 = False
                    self.mode2 = True

                elif self.tof1 > 20:
                    if abs(self.tof2 - self.tof3) > 50 and self.kiri == True:
                        belok = Int32()
                        belok.data = 22
                        self.belok_data.publish(belok)
                        self.flag_belok_capit = True
                        self.capit_flag = True
                        self.get_logger().info("Belok Kiri Ngadep Capit")
                        self.kanan = True
                    elif abs(self.tof2 - self.tof3) > 50 and self.kanan == True:
                        belok = Int32()
                        belok.data = 11
                        self.belok_data.publish(belok)
                        self.flag_belok_capit = True
                        self.flag = 0
                        self.get_logger().info("Belok Kanan")
                        self.flag = self.flag + 1
                    else:
                        maju_mundur = Int32()
                        maju_mundur.data = int(self.gyro[0]) #maju/mundur berapa jauh sama yaw || yaw, jarak, angle
                        self.maju_mundur_data.publish(maju_mundur)
                        self.get_logger().info("Maju")
            

            if self.mode2 == True: 
                if self.flag == 0:
                    #belok kiri 90 derajat
                    belok = Int32()
                    belok.data = 22
                    self.belok_data.publish(belok)
                    self.flag = 1
                elif self.tof1 <= 10 and abs(self.tof2 - self.tof3) > 17 and self.capit_flag == False and self.lepas == True and self.maju == False and self.flag == 0:
                    #belok kiri 90 derajat
                    belok = Int32()
                    belok.data = 22
                    self.belok_data.publish(belok)
                    self.kiri= True
                    self.maju = True
                elif self.tof1 > 20 and self.maju == True:
                    #maju terus sampe kiri kanan beda
                    maju_mundur = Int32()
                    maju_mundur.data = int(self.gyro[0]) #maju/mundur berapa jauh sama yaw || yaw, jarak, angle
                    self.maju_mundur_data.publish(maju_mundur)
                elif self.tof1 < 19:
                    if self.kiri == True:
                        if self.flag == 2:
                            self.kiri = False
                            self.kanan = True
                        if self.flag == 4:
                            self.mode2 = False
                            self.mode3 = True
                        #belok kiri 90 derajat
                        belok = Int32()
                        belok.data = 22
                        self.belok_data.publish(belok)
                        self.flag = self.flag + 1
                    elif self.kanan == True:
                        if self.flag == 4:
                            self.kiri = True
                            self.kanan = False
                        belok = Int32()
                        belok.data = 11
                        self.belok_data.publish(belok)
                        self.flag_belok_capit = True
                        self.flag = self.flag + 1
                        self.get_logger().info("Belok Kanan")



            # if self.mode3 == True:

            # if self.mode4 == True:
                
            # if self.mode5 == True:
            self.tof1_flag = False
            self.tof2_flag = False
            self.tof3_flag = False
        # if self.cam_flag == True or self.gyro_flag == True:
        #     pub = Int32MultiArray()
        #     #self.get_logger().info("Mengirim Yaw: " + str(int(self.gyro[0])) + " Pitch: " + str(int(self.gyro[1])) + " Jarak: " + str(self.cam[0]) + " cm, Sudut: " + str(self.cam[1])) #servo_calculation(self.gyro, self.cam))

        #     bbb = maju_mundur(int(self.gyro[0]), int(self.gyro[1]), int(self.cam[0]), int(self.cam[1]))

        #     self.get_logger().info(str(bbb))

        #     pub.data = bbb
        #     self.publish_data.publish(pub)
            
        #     self.cam_flag = False
        #     self.gyro_flag = False
 
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    GPIO.output(GPIO.LED, GPIO.HIGH)
    while True:
        if GPIO.input(GPIO.BUTTON)==1:
            GPIO.output(GPIO.LED, GPIO.LOW)
            break
    rclpy.spin(node)
    rclpy.shutdown()
