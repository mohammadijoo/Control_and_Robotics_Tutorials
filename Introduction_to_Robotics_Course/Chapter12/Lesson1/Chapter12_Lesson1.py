# publisher_py.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SinePublisher(Node):
    def __init__(self):
        super().__init__('sine_publisher')
        self.pub = self.create_publisher(Float64, 'sine', 10)
        self.k = 0
        self.timer = self.create_timer(0.01, self.step)  # 100 Hz

    def step(self):
        msg = Float64()
        msg.data = float(__import__('math').sin(0.1 * self.k))
        self.pub.publish(msg)
        self.k += 1

def main():
    rclpy.init()
    node = SinePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
      
