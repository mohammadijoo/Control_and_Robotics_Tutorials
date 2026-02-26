#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

class HapticStopNode : public rclcpp::Node {
public:
  HapticStopNode()
  : Node("haptic_stop_node"), force_threshold_(20.0) {  // newton
    using std::placeholders::_1;
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/haptic_force", 10,
      std::bind(&HapticStopNode::force_callback, this, _1)
    );
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void force_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    double Fh = msg->data;
    if (Fh > force_threshold_) {
      geometry_msgs::msg::Twist stop;
      stop.linear.x = 0.0;
      stop.angular.z = 0.0;
      pub_->publish(stop);
      RCLCPP_WARN(this->get_logger(),
                  "High haptic force %.2f N, sending STOP", Fh);
    }
  }

  double force_threshold_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HapticStopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
      
