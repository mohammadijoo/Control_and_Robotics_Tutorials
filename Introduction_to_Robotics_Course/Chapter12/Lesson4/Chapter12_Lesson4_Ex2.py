# ROS2 example listener pattern (usage)
from tf2_ros import Buffer, TransformListener
import rclpy
from rclpy.node import Node

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def query(self):
        try:
            t = self.buffer.lookup_transform(
                'world', 'base_link', rclpy.time.Time())
            self.get_logger().info(str(t.transform))
        except Exception as e:
            self.get_logger().warn(str(e))
      
