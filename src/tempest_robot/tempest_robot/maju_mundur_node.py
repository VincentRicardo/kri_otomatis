import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
import math


coxa = 5
femur = 6
tibia = 13
alpha = 10

def servo_calculation(yaw):
    derajat = [0, 65, 90, 140, 85, 90 ,15, 40, 110, 180, 30, 80, 5, 55, 105, 180, 80, 90, 20]
    #buat kaki tengah
    if yaw > 30:
        yaw = 30
    if yaw < -30:
        yaw = -30
    y = 5  #maju 5 cm, nilai cam distance ngaruh kesini
    x = 11 #+ (17.5*(math.tan((-yaw*math.pi)/180)))
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
        super().__init__("maju_mundur_node")
        self.subscriber_data = self.create_subscription(Int32, "/maju_mundur", self.fungsi_, 1)

        self.publish_data = self.create_publisher(Int32MultiArray, "/state", 1)
      
    def fungsi_(self, message = Int32):
      info = Int32MultiArray()
      info.data = servo_calculation(int(message.data)) #yaw, jarak, angle
      self.publish_data.publish(info)



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
