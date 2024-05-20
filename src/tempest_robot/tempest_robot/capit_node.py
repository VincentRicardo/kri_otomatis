import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import math
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

coxa = 5
femur = 6
tibia = 13
alpha = 10

def servo_calculation(jarak, angle):
    derajat = [0, 65, 90, 140, 85, 105 ,25, 40, 110, 180, 30, 80, 5, 55, 85, 150, 80, 85, 20]
    #buat kaki tengah
    y = 5 + jarak  #maju 5 cm, nilai cam distance ngaruh kesini
    x = 11 + angle
    theta1_Zero = 0
    theta2_Zero = 209.73539429556138
    theta3_Zero = 63.74878606504425
    theta_1 = int(math.degrees(math.atan(y/x)))
    theta_2 = int((math.degrees(math.acos((pow(femur, 2) + pow((math.sqrt(pow (alpha, 2) + pow((math.sqrt(pow(x ,2) + pow(y, 2)) - coxa), 2))), 2) - pow (tibia, 2))/(2 * femur * (math.sqrt(pow (alpha, 2) + pow(math.sqrt(pow(x ,2) + pow(y, 2)) - coxa, 2))))))) + (math.degrees(math.atan((math.sqrt(pow(x ,2) + pow(y, 2)) - coxa)/alpha))) + 90 - theta2_Zero)
    theta_3 = int((math.degrees(math.acos((pow(femur, 2) + pow (tibia, 2) - pow((math.sqrt(pow (alpha, 2) + pow((math.sqrt(pow(x ,2) + pow(y, 2)) - coxa), 2))), 2))/(2 * femur * tibia)))) - theta3_Zero)

    #buat kaki A sama D, cari thetanya zeronya dulu
    yy = 7.77 + 5 + jarak
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
        super().__init__("capit_node")
        self.subscriber_data = self.create_subscription(Int32MultiArray, "/capit", self.fungsi_, 1)

        self.publish_data = self.create_publisher(Int32MultiArray, "/state", 1)
        self.talking_one = self.create_publisher(String, "/flag_belok_capit", 1)

        self.capit_flag = False
      
    def fungsi_(self, message = Int32MultiArray):
      if self.capit_flag == False:
          info = Int32MultiArray()
          info.data = [2, 65, 110, 140, 85, 105 ,25, 40, 110, 180, 30, 80, 5, 55, 85, 150, 80, 65, 15]
          #info.data = servo_calculation(message.data[0], message.data[1]) # jarak, angle
          self.publish_data.publish(info)
          tahan_waktu(3)
          info.data = servo_calculation(int(message.data[0]), int(message.data[1]))
          self.publish_data.publish(info)
          tahan_waktu(3)
          info.data = [4, 65, 90, 140, 85, 105 ,25, 40, 110, 180, 30, 80, 5, 55, 85, 150, 80, 85, 15]
          self.publish_data.publish(info)
          tahan_waktu(3)
          info.data = servo_calculation(int(message.data[0]), 0)
          self.publish_data.publish(info)

          self.capit_flag = True
          copy = String()
          copy.data ="done"
          self.talking_one.publish(copy)
      elif self.capit_flag == True:
          info = Int32MultiArray()
          info.data = [4, 65, 90, 140, 85, 105 ,25, 40, 110, 180, 30, 80, 5, 55, 85, 150, 80, 85, 15]
          self.publish_data.publish(info)
          copy = String()
          copy.data ="done"
          self.talking_one.publish(copy)
      



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()