#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class DebugController : public rclcpp::Node {
public:
  DebugController() : Node("debug_controller"), ref_(0.0f), y_(0.0f),
                      gamma_(0.5f) {
    ref_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "ref", 10, std::bind(&DebugController::refCallback, this, _1));
    y_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "y", 10, std::bind(&DebugController::yCallback, this, _1));
    last_time_ = now();
    timer_ = this->create_wall_timer(
      20ms, std::bind(&DebugController::controlStep, this));
  }

private:
  void refCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    ref_ = msg->data;
  }

  void yCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    y_ = msg->data;
  }

  void controlStep() {
    auto current = now();
    double dt = (current - last_time_).seconds();
    last_time_ = current;

    if (dt > 0.05) {
      RCLCPP_WARN(this->get_logger(), "Loop overrun: dt=%.3f s", dt);
    }

    float e = ref_ - y_;
    if (std::fabs(e) > gamma_) {
      RCLCPP_WARN(this->get_logger(), "Tracking error too large: e=%.3f", e);
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ref_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr y_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  float ref_;
  float y_;
  float gamma_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugController>());
  rclcpp::shutdown();
  return 0;
}
      
