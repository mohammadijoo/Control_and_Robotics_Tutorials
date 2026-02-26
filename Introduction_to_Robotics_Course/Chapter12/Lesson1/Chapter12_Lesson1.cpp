// subscriber_cpp.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class SineSubscriber : public rclcpp::Node {
public:
  SineSubscriber() : Node("sine_subscriber") {
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "sine", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "sine=%f", msg->data);
      }
    );
  }
private:
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SineSubscriber>());
  rclcpp::shutdown();
  return 0;
}
      
