import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.sub = self.create_subscription(
            String, 'chatter', self.cb, 10)

    def cb(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main():
    rclpy.init()
    rclpy.spin(SimpleSubscriber())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
      
