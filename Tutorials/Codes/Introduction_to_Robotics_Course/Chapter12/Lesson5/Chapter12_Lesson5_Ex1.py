# controller_pkg/controller_pkg/controller_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.sub = self.create_subscription(Float64, '/sensor', self.cb, 10)
        self.pub = self.create_publisher(Float64, '/cmd', 10)

        self.Kp = 1.5
        self.r = 1.0  # step reference

    def cb(self, msg):
        y = msg.data
        u = self.Kp * (self.r - y)
        out = Float64()
        out.data = u
        self.pub.publish(out)

def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
