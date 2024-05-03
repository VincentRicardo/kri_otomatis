import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import math

from getkey import getkey, keys
import RPi.GPIO as GPIO

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


def maju_mundur(yaw, pitch, jarak, angle):
    derajat = [0, 65, 90, 140, 85, 105 ,25, 40, 110, 180, 30, 80, 5, 55, 85, 150, 80, 85, 15]
    #buat kaki tengah
    if yaw > 10:
        yaw = 10
    y = 5  #maju 3 cm, nilai cam distance ngaruh kesini
    x = 11 #lebarnya, nilai gyro yaw dan angle cam ngaruh kesini
    theta1_Zero = 0
    theta2_Zero = 209.73539429556138
    theta3_Zero = 63.74878606504425
    theta_1 = int(math.degrees(math.atan(y/x)))
    theta_2 = int((math.degrees(math.acos((pow(femur, 2) + pow((math.sqrt(pow (alpha, 2) + pow((math.sqrt(pow(x ,2) + pow(y, 2)) - coxa), 2))), 2) - pow (tibia, 2))/(2 * femur * (math.sqrt(pow (alpha, 2) + pow(math.sqrt(pow(x ,2) + pow(y, 2)) - coxa, 2))))))) + (math.degrees(math.atan((math.sqrt(pow(x ,2) + pow(y, 2)) - coxa)/alpha))) + 90 - theta2_Zero)
    theta_3 = int((math.degrees(math.acos((pow(femur, 2) + pow (tibia, 2) - pow((math.sqrt(pow (alpha, 2) + pow((math.sqrt(pow(x ,2) + pow(y, 2)) - coxa), 2))), 2))/(2 * femur * tibia)))) - theta3_Zero)

    #buat kaki A sama D, cari thetanya zeronya dulu
    yy = 7.77 + 5
    xx = 7.77 
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
    degree.append(derajat[4] - thetaa_1)
    degree.append(derajat[5] + thetaa_2)
    degree.append(derajat[6] - thetaa_3)

    #E
    degree.append(derajat[7] + theta_1)
    degree.append(derajat[8] + theta_2)
    degree.append(derajat[9] + theta_3)
    #B
    degree.append(derajat[10] - theta_1)
    degree.append(derajat[11] - theta_2)
    degree.append(derajat[12] - theta_3)
    
    #D
    degree.append(derajat[13] + thetaa_1)
    degree.append(derajat[14] - thetaa_2)
    degree.append(derajat[15] + thetaa_3)
    #F
    degree.append(derajat[16] + thetaa_1)
    degree.append(derajat[17] - thetaa_2)
    degree.append(derajat[18] + thetaa_3)

    
    return degree
class MyNode(Node):
    def __init__(self):
        super().__init__("olah_node")
        self.gyro_flag = False
        self.cam_flag = False
        self.first_flag = False

        self.capit_flag = False

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
        self.subscriber_tof1 = self.create_subscription(Int32, "/tof1_", self.tof1_, 1)
        self.subscriber_tof1 = self.create_subscription(Int32, "/tof2_", self.tof2_, 1)
        self.subscriber_tof1 = self.create_subscription(Int32, "/tof3_", self.tof3_, 1)


        self.publish_data = self.create_publisher(Int32MultiArray, "/state", 1)

        self.timer_ = self.create_timer(0.1, self.send_mode_call)
        self.first_ = self.create_timer(5, self.send_first)

        self.cam = [0,0]
        self.gyro = [0,0]
        self.tof1 = 0
        

    
    def cam_(self, message = Int32MultiArray):
        self.get_logger().info("Receiving Cam Distance = " + str(message.data[0]) + " cm & Angle = " + str(message.data[1]))
        
        self.cam = [message.data[0], message.data[1]]

        self.cam_flag = True

    def gyro_(self, message = Int32MultiArray):
        self.get_logger().info("Receiving Yaw = " + str(message.data[0])+ " & Pitch = " + str(message.data[1]))
        self.gyro = [message.data[0], message.data[1]]
        self.gyro_flag = True
        self.first_flag = True

    #depan robot
    def tof1_ (self, message = Int32):
        self.get_logger().info("Receiving ToF 1 Distance: " + str(message.data) + " cm")
        self.tof1 = message.data
        self.tof1_flag = True
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
    

    def send_first(self):
        if self.first_flag == False:
            ddd = Int32MultiArray()
            ddd.data = [1, 65, 90, 140, 85, 105 ,25, 40, 110, 180, 30, 80, 5, 55, 85, 150, 80, 85, 15]
            self.publish_data.publish(ddd)


    def send_mode_call(self):
        if self.gyro_flag == True and self.tof1_flag == True and self.tof2_flag == True and self.tof3_flag == True:
            if self.mode1 == True:
                if self.tof1 == 3 and self.tof2 == 3 and self.capit_flag == False:
                    #belok kanan 90 derajat
                    
                elif self.tof1 == 3 and self.tof3 == 3 and self.capit_flag == False:
                    #belok kiri 90 derajat
                    
                elif abs(self.tof2 - self.tof3) < 5:
                    #maju terus sampe kiri kanan beda
                    
                elif abs (self.tof2 - self.tof3) >= 5:
                    #belok kiri 90 derajat terus masuk ke mode pencapitan
                    self.capit_flag == True
                    
                elif self.capit_flag == True and self.tof1 == 3 and self.tof3 == 3:
                    #belok kanan 90 derajat trus lepas pencapitan
                    # belok kiri 90 derajat
                    self.mode1 = False
                    self.mode2 = True

            if self.mode2 == True:
                
            if self.mode3 == True:

            if self.mode4 == True:
                
            if self.mode5 == True:

        if self.cam_flag == True or self.gyro_flag == True:
            pub = Int32MultiArray()
            #self.get_logger().info("Mengirim Yaw: " + str(int(self.gyro[0])) + " Pitch: " + str(int(self.gyro[1])) + " Jarak: " + str(self.cam[0]) + " cm, Sudut: " + str(self.cam[1])) #servo_calculation(self.gyro, self.cam))

            bbb = maju_mundur(int(self.gyro[0]), int(self.gyro[1]), int(self.cam[0]), int(self.cam[1]))

            self.get_logger().info(str(bbb))

            pub.data = bbb
            self.publish_data.publish(pub)
            
            self.cam_flag = False
            self.gyro_flag = False
 
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    print("Menunggu button")
    GPIO.output(GPIO.LED, GPIO.HIGH)
    while True:
        if GPIO.input(GPIO.BUTTON)==1:
            print("Sistem Aktif")
            GPIO.output(GPIO.LED, GPIO.LOW)
            break
    rclpy.spin(node)
    rclpy.shutdown()