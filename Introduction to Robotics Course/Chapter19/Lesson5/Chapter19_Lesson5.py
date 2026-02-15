import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DebugController(Node):
    def __init__(self):
        super().__init__('debug_controller')
        self.ref = 0.0
        self.y = 0.0
        self.gamma = 0.5  # threshold for |e_k|
        self.last_time = self.get_clock().now()

        self.create_subscription(Float32, 'ref', self.ref_callback, 10)
        self.create_subscription(Float32, 'y', self.y_callback, 10)
        self.timer = self.create_timer(0.02, self.control_step)  # 50 Hz

    def ref_callback(self, msg):
        self.ref = msg.data

    def y_callback(self, msg):
        self.y = msg.data

    def control_step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt > 0.05:
            self.get_logger().warn(f'Loop overrun: dt={dt:.3f} s')

        e = self.ref - self.y
        if abs(e) > self.gamma:
            self.get_logger().warn(f'Tracking error too large: e={e:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = DebugController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
      
