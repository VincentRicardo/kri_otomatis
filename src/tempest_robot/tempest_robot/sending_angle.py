import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nano0servo_interfaces.action import SendAngleN0

from std_msgs.msg import Int32MultiArray
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

class SendAngle0ClientNode(Node):
    def __init__(self):
        super().__init__("sending_angle")
        self.subscriber_sudut0 = self.create_subscription(Int32MultiArray, "/state", self.data_, 1)
        self.send_angle0_client = ActionClient(self, SendAngleN0, "nano0_action")
        self.send_angle1_client = ActionClient(self, SendAngleN0, "nano1_action")

    def data_ (self, message: Int32MultiArray):
        #Wait for server
        self.send_angle0_client.wait_for_server()

        #Create a goal
        goal = SendAngleN0.Goal()
        for i in range(19):
            goal.servo[i] = message.data[i]


        #send the goal
        self.get_logger().info("Sending Angle 0 Data")
        self.send_angle0_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        self.send_angle1_client.wait_for_server()

        goal = SendAngleN0.Goal()
        result = future.result().result
        goal.servo[0] = result.servoo[0]
        for i in range(10, 19):
            goal.servo[i] = result.servoo[i-9]
        #send the goal
        self.get_logger().info("Sending Angle 1 Data")
        self.send_angle1_client.send_goal_async(goal).add_done_callback(self.goal_response_callback2)
    
    def goal_response_callback2(self, future):
        self.goal_handle2_: ClientGoalHandle = future.result()
        if self.goal_handle2_.accepted:
            self.goal_handle2_.get_result_async().add_done_callback(self.goal_result_callback2)
    
    def goal_result_callback2(self, future):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SendAngle0ClientNode()
    rclpy.spin(node)
    rclpy.shutdown()