// ROS2 C++ example: seed passed as parameter
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <random>

class NoisyPublisher : public rclcpp::Node {
public:
  NoisyPublisher() : Node("noisy_pub") {
    this->declare_parameter<int>("seed", 0);
    int seed = this->get_parameter("seed").as_int();

    rng_.seed(seed); // deterministic PRG

    pub_ = this->create_publisher<std_msgs::msg::Float32>("noise", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NoisyPublisher::step, this));
  }
private:
  void step() {
    std::normal_distribution<float> dist(0.0f, 1.0f);
    float z = dist(rng_);
    std_msgs::msg::Float32 msg;
    msg.data = z;
    pub_->publish(msg);
  }
  std::mt19937 rng_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
      
