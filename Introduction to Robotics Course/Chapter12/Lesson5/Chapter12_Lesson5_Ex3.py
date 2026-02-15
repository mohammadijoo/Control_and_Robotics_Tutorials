# tf_pkg/tf_pkg/tf_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class TFNode(Node):
    def __init__(self):
        super().__init__('tf_node')
        self.broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # identity quaternion

        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TFNode()
    rclpy.spin(node)
    rclpy.shutdown()
