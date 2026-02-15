import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class TFPublisher(Node):
    def __init__(self):
        super().__init__("ros2_tf_publisher")
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.033, self.tick)

    def tick(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu_frame"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05

        yaw = 0.0
        t.transform.rotation.z = math.sin(yaw/2)
        t.transform.rotation.w = math.cos(yaw/2)

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = TFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
      
