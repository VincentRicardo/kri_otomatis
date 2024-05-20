from launch import LaunchDescription
import launch_ros.actions

import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.XSHUT_DEPAN = 13
GPIO.XSHUT_KIRI = 19
GPIO.XSHUT_KANAN = 26


GPIO.setup(GPIO.XSHUT_DEPAN, GPIO.OUT)
GPIO.setup(GPIO.XSHUT_KANAN, GPIO.OUT)
GPIO.setup(GPIO.XSHUT_KIRI, GPIO.OUT)

GPIO.output(GPIO.XSHUT_KIRI, GPIO.LOW)
GPIO.output(GPIO.XSHUT_KANAN, GPIO.LOW)
GPIO.output(GPIO.XSHUT_DEPAN, GPIO.LOW)


#depan
GPIO.output(GPIO.XSHUT_DEPAN, GPIO.HIGH)
i2c = busio.I2C(board.SCL, board.SDA)
vl531_depan = adafruit_vl53l0x.VL53L0X(i2c, 41)
vl531_depan.set_address(48)

#kiri
GPIO.output(GPIO.XSHUT_KIRI, GPIO.HIGH)
i2c = busio.I2C(board.SCL, board.SDA)
vl531_kiri = adafruit_vl53l0x.VL53L0X(i2c, 41)
vl531_kiri.set_address(49)

#kanan
GPIO.output(GPIO.XSHUT_KANAN, GPIO.HIGH)
i2c = busio.I2C(board.SCL, board.SDA)
vl531_kanan = adafruit_vl53l0x.VL53L0X(i2c, 41)
vl531_kanan.set_address(50)



def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package = 'tempest_robot', executable= 'nano0_server'),
        launch_ros.actions.Node(
            package = 'tempest_robot', executable = 'nano1_server'),
        launch_ros.actions.Node(
            package = 'tempest_robot', executable = 'sending_angle'),
        launch_ros.actions.Node(
            package = 'tempest_robot', executable = 'gyro_node'),
        launch_ros.actions.Node(
            package = 'tempest_robot', executable = 'camera_node'),
        launch_ros.actions.Node(
            package = 'tempest_robot', executable = 'capit_node'),
        launch_ros.actions.Node(
            package = 'tempest_robot', executable = 'maju_mundur_node'),
        launch_ros.actions.Node(
            package = 'tempest_robot', executable = 'belok_node'),
        launch_ros.actions.Node(
            package = 'tempest_robot', executable = 'tof1_node'),
        # launch_ros.actions.Node(
        #     package = 'tempest_robot', executable = 'tof2_node'),
        # launch_ros.actions.Node(
        #     package = 'tempest_robot', executable = 'tof3_node'),
        launch_ros.actions.Node(
            package = 'tempest_robot', executable = 'olah_node')
        ])
