# ROS2 Python node with reproducible randomness
import rclpy
from rclpy.node import Node
import numpy as np

class NoisyPublisher(Node):
    def __init__(self):
        super().__init__('noisy_pub')
        self.declare_parameter('seed', 0)
        seed = self.get_parameter('seed').value
        self.rng = np.random.default_rng(seed)  # deterministic PRG

        self.pub = self.create_publisher(Float32, 'noise', 10)
        self.timer = self.create_timer(0.1, self.step)

    def step(self):
        z = float(self.rng.normal(0.0, 1.0))
        msg = Float32()
        msg.data = z
        self.pub.publish(msg)
        self.get_logger().info(f"noise={z:.3f}")

def main():
    rclpy.init()
    node = NoisyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
      
