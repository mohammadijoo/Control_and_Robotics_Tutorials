#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>

class SimpleRuleBasedController : public rclcpp::Node {
public:
  SimpleRuleBasedController()
  : Node("simple_rule_based_controller"),
    d_front_(std::numeric_limits<double>::quiet_NaN()),
    d_right_(std::numeric_limits<double>::quiet_NaN())
  {
    d_front_safe_ = 0.5;
    d_right_ref_ = 0.6;
    v_cruise_ = 0.2;
    omega_turn_ = 0.6;
    k_omega_ = 1.0;

    sub_front_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/front_range", 10,
      std::bind(&SimpleRuleBasedController::frontCallback, this, std::placeholders::_1));

    sub_right_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/right_range", 10,
      std::bind(&SimpleRuleBasedController::rightCallback, this, std::placeholders::_1));

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SimpleRuleBasedController::controlLoop, this));
  }

private:
  void frontCallback(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    d_front_ = msg->range;
  }

  void rightCallback(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    d_right_ = msg->range;
  }

  void controlLoop()
  {
    if (!std::isfinite(d_front_)) {
      return;
    }

    double v = 0.0;
    double omega = 0.0;

    if (d_front_ < d_front_safe_) {
      v = 0.0;
      omega = omega_turn_;
    } else if (std::isfinite(d_right_)) {
      v = v_cruise_;
      double error_right = d_right_ - d_right_ref_;
      omega = k_omega_ * error_right;
    } else {
      v = v_cruise_;
      omega = 0.0;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = omega;
    pub_cmd_->publish(cmd);
  }

  // Parameters
  double d_front_safe_;
  double d_right_ref_;
  double v_cruise_;
  double omega_turn_;
  double k_omega_;

  // Measurements
  double d_front_;
  double d_right_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_front_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_right_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleRuleBasedController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
      
