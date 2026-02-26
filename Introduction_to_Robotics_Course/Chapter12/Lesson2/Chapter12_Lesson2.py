import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SinePublisher(Node):
    def __init__(self):
        super().__init__('sine_publisher')
        self.pub = self.create_publisher(Float64, '/u_cmd', 10)
        self.t = 0.0
        self.timer = self.create_timer(0.01, self.step)  # 100 Hz

    def step(self):
        msg = Float64()
        msg.data = 1.0  # placeholder input
        self.pub.publish(msg)

class UListener(Node):
    def __init__(self):
        super().__init__('u_listener')
        self.sub = self.create_subscription(
            Float64, '/u_cmd', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f"u_cmd={msg.data}")

def main():
    rclpy.init()
    pub_node = SinePublisher()
    sub_node = UListener()
    executor = rclpy.executors.MultiExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)
    executor.spin()

if __name__ == "__main__":
    main()
      
