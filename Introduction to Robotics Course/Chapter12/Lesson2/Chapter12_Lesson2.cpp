#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class UCmdPub : public rclcpp::Node {
public:
  UCmdPub() : Node("u_cmd_pub") {
    pub_ = create_publisher<std_msgs::msg::Float64>("/u_cmd", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&UCmdPub::step, this)
    );
  }
private:
  void step() {
    std_msgs::msg::Float64 msg;
    msg.data = 1.0;
    pub_->publish(msg);
  }
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
      
