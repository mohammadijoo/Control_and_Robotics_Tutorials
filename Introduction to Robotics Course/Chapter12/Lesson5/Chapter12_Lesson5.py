# sensor_pkg/sensor_pkg/sensor_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math, random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.pub = self.create_publisher(Float64, '/sensor', 10)
        self.Ts = 0.02  # 50 Hz
        self.k = 0
        self.timer = self.create_timer(self.Ts, self.tick)

    def tick(self):
        # simple scalar signal y_k = sin(0.5 t) + noise
        t = self.k * self.Ts
        y = math.sin(0.5 * t) + random.gauss(0.0, 0.01)
        msg = Float64()
        msg.data = y
        self.pub.publish(msg)
        self.k += 1

def main():
    rclpy.init()
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
