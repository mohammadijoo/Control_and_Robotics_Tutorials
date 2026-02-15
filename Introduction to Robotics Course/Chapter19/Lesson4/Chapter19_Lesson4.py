import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class SimpleRuleBasedController(Node):
    def __init__(self):
        super().__init__('simple_rule_based_controller')
        # Parameters
        self.d_front_safe = 0.5     # [m]
        self.d_right_ref = 0.6      # [m]
        self.v_cruise = 0.2         # [m/s]
        self.omega_turn = 0.6       # [rad/s]
        self.k_omega = 1.0          # proportional gain

        # Internal measurements
        self.d_front = None
        self.d_right = None

        # Subscriptions and publisher
        self.create_subscription(
            Range, '/front_range', self.front_callback, 10)
        self.create_subscription(
            Range, '/right_range', self.right_callback, 10)
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # Control loop timer
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def front_callback(self, msg: Range):
        self.d_front = msg.range

    def right_callback(self, msg: Range):
        self.d_right = msg.range

    def control_loop(self):
        # Only act when we have at least the front measurement
        if self.d_front is None:
            return

        v = 0.0
        omega = 0.0

        # Rule 1: if front is too close, stop and turn
        if self.d_front < self.d_front_safe:
            v = 0.0
            omega = self.omega_turn

        # Rule 2: if front is clear and we have a right wall, follow it
        elif self.d_right is not None:
            v = self.v_cruise
            error_right = self.d_right - self.d_right_ref
            omega = self.k_omega * error_right

        # Rule 3: otherwise, just cruise forward
        else:
            v = self.v_cruise
            omega = 0.0

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRuleBasedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
      
