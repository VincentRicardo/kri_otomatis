from launch import LaunchDescription
import launch_ros.actions

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
            package = 'tempest_robot', executable = 'olah_node')
        ])
