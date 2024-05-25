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

coxa = 5
femur = 6
tibia = 13
alpha = 10


def servo_calculation(geser):
    derajat = [0, 65, 90, 180, 85, 90 ,0, 40, 100, 180, 30, 80, 5, 55, 105, 180, 80, 100, 0]
    #buat kaki tengah

    y = 0 #maju 5 cm, nilai cam distance ngaruh kesini
    x = 11 + geser #+ (17.5*(math.tan((-yaw*math.pi)/180)))
    theta1_Zero = 0
    theta2_Zero = 199.4257715524105
    theta3_Zero = 72.07978686060771
    theta_1 = int(math.degrees(math.atan(y/x)))
    theta_2 = int((math.degrees(math.acos((pow(femur, 2) + pow((math.sqrt(pow (alpha, 2) + pow((math.sqrt(pow(x ,2) + pow(y, 2)) - coxa), 2))), 2) - pow (tibia, 2))/(2 * femur * (math.sqrt(pow (alpha, 2) + pow(math.sqrt(pow(x ,2) + pow(y, 2)) - coxa, 2))))))) + (math.degrees(math.atan((math.sqrt(pow(x ,2) + pow(y, 2)) - coxa)/alpha))) + 90 - theta2_Zero)
    theta_3 = int((math.degrees(math.acos((pow(femur, 2) + pow (tibia, 2) - pow((math.sqrt(pow (alpha, 2) + pow((math.sqrt(pow(x ,2) + pow(y, 2)) - coxa), 2))), 2))/(2 * femur * tibia)))) - theta3_Zero)

    #buat kaki A sama D, cari thetanya zeronya dulu
    yy = 7.77  
    xx = 7.77 + geser
    theta1_2Zero = 45

    thetaa_1 = int((math.degrees(math.atan(yy/xx))) - theta1_2Zero)
    thetaa_2 = int((math.degrees(math.acos((pow(femur, 2) + pow((math.sqrt(pow (alpha, 2) + pow((math.sqrt(pow(xx ,2) + pow(yy, 2)) - coxa), 2))), 2) - pow (tibia, 2))/(2 * femur * (math.sqrt(pow (alpha, 2) + pow(math.sqrt(pow(xx ,2) + pow(yy, 2)) - coxa, 2))))))) + (math.degrees(math.atan((math.sqrt(pow(xx, 2) + pow(yy, 2)) - coxa)/alpha))) + 90 - theta2_Zero)
    thetaa_3 = int((math.degrees(math.acos((pow(femur, 2) + pow (tibia, 2) - pow((math.sqrt(pow (alpha, 2) + pow((math.sqrt(pow(xx, 2) + pow(yy, 2)) - coxa), 2))), 2))/(2 * femur * tibia)))) - theta3_Zero)

    degree = []
    degree.append(0)
    #A
    degree.append(derajat[1] - thetaa_1)
    degree.append(derajat[2] + thetaa_2)
    degree.append(derajat[3] - thetaa_3)
    #C
    degree.append(derajat[4] + thetaa_1)
    degree.append(derajat[5] - thetaa_2)
    degree.append(derajat[6] + thetaa_3)

    #E
    degree.append(derajat[7] + theta_1)
    degree.append(derajat[8] + theta_2)
    degree.append(derajat[9] - theta_3)
    #B
    degree.append(derajat[10] + theta_1)
    degree.append(derajat[11] - theta_2)
    degree.append(derajat[12] + theta_3)
    
    #D
    degree.append(derajat[13] + thetaa_1)
    degree.append(derajat[14] + thetaa_2)
    degree.append(derajat[15] - thetaa_3)
    #F
    degree.append(derajat[16] - thetaa_1)
    degree.append(derajat[17] - thetaa_2)
    degree.append(derajat[18] + thetaa_3)

    return degree


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

        self.flag = 111
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
        #self.subscriber_tof2 = self.create_subscription(Int32, "/tof2_", self.tof2_, 1)
        #self.subscriber_tof3 = self.create_subscription(Int32, "/tof3_", self.tof3_, 1)


        self.publish_data = self.create_publisher(Int32MultiArray, "/state", 1)
        self.maju_mundur_data = self.create_publisher(Int32MultiArray, "/maju_mundur", 1)
        self.belok_data = self.create_publisher(Int32, "/belok", 1)
        self.capit_data = self.create_publisher(Int32, "/capit", 1)

        self.timer_ = self.create_timer(0.1, self.send_mode_call)
        self.first_ = self.create_timer(5, self.send_first)

        self.cam = [0,0]
        self.gyro = [0,0]
        self.tof1 = 0
        self.tof2 = 0
        self.tof3 = 0
        

    
    def cam_(self, message = Int32MultiArray):
        self.get_logger().info("Receiving Cam Distance = " + str(message.data[0]) + " cm & Angle = " + str(message.data[1]))
        
        self.cam = [int(message.data[0]), int(message.data[1])]

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
        
    # #sebelah kiri robot
    # def tof2_ (self, message = Int32):
    #     self.get_logger().info("Receiving ToF 2 Distance: " + str(message.data) + " cm")
    #     self.tof2 = message.data
    #     self.tof2_flag = True
        
    # #sebelah kanan robot
    # def tof3_ (self, message = Int32):
    #     self.get_logger().info("Receiving ToF 3 Distance: " + str(message.data) + " cm")
    #     self.tof3 = message.data
    #     self.tof3_flag = True
    
    def flag_function(self, message:String):
        self.flag_belok_capit = False

    def send_first(self):
        if self.first_flag == False:
            ddd = Int32MultiArray()
            ddd.data = [1, 65, 90, 180, 85, 90 ,0, 40, 100, 180, 30, 80, 5, 55, 105, 180, 80, 100, 0]
            self.publish_data.publish(ddd)
            self.first_flag = True


    def send_mode_call(self):
        if self.gyro_flag == True and self.tof1_flag == True and self.tof2_flag == True and self.tof3_flag == True and self.flag_belok_capit == False:
            #maju_mundur = Int32MultiArray()
            #maju_mundur.data = [5, int(self.gyro[0])] #maju/mundur berapa jauh sama yaw || yaw, jarak, angle
            #self.maju_mundur_data.publish(maju_mundur)
            #self.get_logger().info("Maju")
            if self.mode1 == True:
                if self.tof1 <= 20 and self.tof2 <= 30 and self.flag == 111:
                    #belok kanan 90 derajat

                    belok = Int32()
                    belok.data = 11
                    self.belok_data.publish(belok)
                    self.flag_belok_capit = True
                    self.get_logger().info("Ngadep Kanan")
                    self.flag = 0
                    
                elif self.tof1 <= 20 and self.tof3 <= 30 and self.flag == 111:
                    #belok kiri 90 derajat
                    
                    belok = Int32()
                    belok.data = 22
                    self.belok_data.publish(belok)
                    self.flag_belok_capit = True
                    self.get_logger().info("Ngadep Kiri")
                    self.flag = 0
                
                elif self.capit_flag == True and self.lepas == True and self.flag == 0:
                    #nyapit
                    info = Int32MultiArray()
                    if self.cam[1] < -4: #geser kanan
                        info.data = servo_calculation(4) #ganti flag depannya
                        info.data[0] = int(6)
                        self.publish_data.publish(info)
                    elif self.cam[1] > 4: #geser kiri
                        info.data = servo_calculation(4) #ganti flag depannya
                        info.data[0] = int(5)
                        self.publish_data.publish(info)
                    else:
                        if self.cam[0] > 5:
                            maju_mundur = Int32MultiArray()
                            maju_mundur.data = [3, 0] #maju/mundur berapa jauh sama yaw || yaw, jarak, angle
                            self.maju_mundur_data.publish(maju_mundur)
                            self.get_logger().info("Maju")
                        else:
                            dataa= Int32()
                            dataa.data = int(self.cam[0])
                            self.capit_data.publish(dataa)
                            self.flag_belok_capit = True
                            self.lepas == False
                            self.capit_flag = False
                            self.get_logger().info("Nyapit")
                elif self.flag == 2:
                    data = Int32()
                    data.data = 3 #jarak
                    self.capit_data.publish(data)
                    self.flag_belok_capit = True
                    self.lepas == True
                    self.get_logger().info("Lepas Nyapit")
                    self.flag = 0
                    self.kanan = False
                    self.kiri = True
                    self.mode1 = False
                    self.mode2 = True

                elif self.tof1 > 10 or self.kiri == False:
                    if abs((self.tof2 + self.tof3)/2) > 15 and self.kiri == True and self.flag > 16:
                        #belok = Int32()
                        #belok.data = 22
                        #self.belok_data.publish(belok)
                        ddd = Int32MultiArray()
                        ddd.data = [1, 65, 90, 140, 85, 90 ,15, 40, 110, 180, 30, 80, 5, 55, 105, 180, 80, 100, 20]
                        self.publish_data.publish(ddd) 
                        #self.flag_belok_capit = True
                        #self.capit_flag = True
                        self.get_logger().info("Belok Kiri Ngadep Capit")
                        self.kiri = False
                        self.kanan = True
                        self.flag = 0
                    elif abs((self.tof2 + self.tof3)/2) > 15 and self.kanan == True:
                        #belok = Int32()
                        #belok.data = 11
                        #self.belok_data.publish(belok)
                        ddd = Int32MultiArray()
                        ddd.data = [1, 65, 90, 140, 85, 90 ,15, 40, 110, 180, 30, 80, 5, 55, 105, 180, 80, 100, 20]
                        self.publish_data.publish(ddd)
                        #self.flag_belok_capit = True
                        self.get_logger().info("Belok Kanan")
                        self.flag = self.flag + 1
                        self.kanan = False
                    else:
                        if abs((self.tof2 + self.tof3)/2) > 15 and self.kiri == True:
                            self.flag = self.flag + 5
                        maju_mundur = Int32MultiArray()
                        maju_mundur.data = [5, int(self.gyro[0])] #maju/mundur berapa jauh sama yaw || yaw, jarak, angle
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
