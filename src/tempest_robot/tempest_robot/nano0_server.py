import rclpy
from rclpy.node import Node
from nano0servo_interfaces.action import SendAngleN0
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from std_msgs.msg import String
import time
import serial

flag = True
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
myTime = time.time()
while(flag == True):
    if(time.time() - myTime < 2):
        flag = True
    elif(time.time() - myTime >= 2):
        flag = False

class SendAngle0ServerNode(Node):
    def __init__(self):
        super().__init__("nano0_server")
        self.send_angle0_server = ActionServer(self, SendAngleN0, "nano0_action", execute_callback = self.sending_angle0)
        self.get_logger().info("Action server 0 has been started")
    
    def sending_angle0(self, goal_handle: ServerGoalHandle):
        #Get request
        #dat = []
        dat = [0] * 10
        for i in range(10):
            dat[i] = goal_handle.request.servo[i]

        #Execute the action
        self.get_logger().info("Sending Angle to Nano 0")
        # ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        # sleep(2)
        ser.reset_input_buffer()
        kalimat = "{} {} {} {} {} {} {} {} {} {}".format(dat[0], dat[1], dat[2], dat[3], dat[4], dat[5], dat[6], dat[7], dat[8], dat[9])
        ser.write(kalimat.encode())
        self.get_logger().info(kalimat)
        # line = ser.readline().decode().strip()
        # self.get_logger().info(line)

        #once done, set goal final state
        while True:
            numbercheck = ser.read()
            if numbercheck != b'':
                if int.from_bytes(numbercheck, byteorder = 'big') == 18:
                    goal_handle.succeed()
                    break

        #and send the result
        result = SendAngleN0.Result()
        # result.send = True
        result.servoo[0] = goal_handle.request.servo[0]

        for i in range (10, 19):
            result.servoo[i-9] = goal_handle.request.servo[i]
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SendAngle0ServerNode()
    rclpy.spin(node)
    rclpy.shutdown()
