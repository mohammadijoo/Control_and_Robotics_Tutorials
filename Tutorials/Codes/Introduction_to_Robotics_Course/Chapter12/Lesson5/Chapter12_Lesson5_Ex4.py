# controller_pkg/launch/minimal_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sensor_pkg', executable='sensor_node', name='sensor_node'),
        Node(package='controller_pkg', executable='controller_node', name='controller_node'),
        Node(package='motor_pkg', executable='motor_node', name='motor_node'),
        Node(package='tf_pkg', executable='tf_node', name='tf_node'),
    ])
