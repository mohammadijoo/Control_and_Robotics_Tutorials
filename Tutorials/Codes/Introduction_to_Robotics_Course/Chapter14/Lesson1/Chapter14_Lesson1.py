import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SpeechCommandNode(Node):
    def __init__(self):
        super().__init__("speech_command_node")
        self.sub = self.create_subscription(
            String, "/speech_cmd", self.speech_callback, 10
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # simple mapping from keywords to linear velocity (m/s)
        self.command_table = {
            "forward": 0.3,
            "back": -0.3,
            "stop": 0.0,
        }

    def speech_callback(self, msg: String):
        text = msg.data.lower()
        v = 0.0
        for keyword, v_cmd in self.command_table.items():
            if keyword in text:
                v = v_cmd
                break

        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = 0.0
        self.pub.publish(twist)
        self.get_logger().info(f"Speech: '{text}' - cmd v = {v:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
      
