// controller_pkg/src/controller_node.cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class ControllerNode : public rclcpp::Node {
public:
  ControllerNode() : Node("controller_node"), Kp_(1.5), r_(1.0) {
    sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/sensor", 10,
      std::bind(&ControllerNode::cb, this, std::placeholders::_1)
    );
    pub_ = this->create_publisher<std_msgs::msg::Float64>("/cmd", 10);
  }

private:
  void cb(const std_msgs::msg::Float64::SharedPtr msg) {
    double y = msg->data;
    double u = Kp_ * (r_ - y);
    std_msgs::msg::Float64 out;
    out.data = u;
    pub_->publish(out);
  }

  double Kp_, r_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
