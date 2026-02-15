# motor_pkg/motor_pkg/motor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.sub = self.create_subscription(Float64, '/cmd', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f"Actuating with u={msg.data:.3f}")

def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
