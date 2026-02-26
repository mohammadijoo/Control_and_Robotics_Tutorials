import rclpy
from rclpy.node import Node

SAFE_DISTANCE = 0.8  # meters (example only)
SPEED_LIMIT = 0.25   # m/s

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor")
        self.speed = 0.0
        self.distance = float("inf")

        self.create_subscription(
            Float64, "/robot/speed", self.speed_callback, 10
        )
        self.create_subscription(
            Float64, "/human/distance", self.distance_callback, 10
        )
        self.estop_pub = self.create_publisher(Bool, "/safety/estop", 10)
        self.create_timer(0.01, self.check_safety)  # 100 Hz monitor

    def speed_callback(self, msg):
        self.speed = msg.data

    def distance_callback(self, msg):
        self.distance = msg.data

    def check_safety(self):
        # This simple rule mimics a subset of ISO/TS 15066 style thinking:
        # if human is close AND speed is high, stop.
        if self.distance <= SAFE_DISTANCE and self.speed > SPEED_LIMIT:
            self.get_logger().warn("Safety violation detected, triggering E-STOP")
            self.estop_pub.publish(Bool(data=True))

def main():
    rclpy.init()
    node = SafetyMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
      
