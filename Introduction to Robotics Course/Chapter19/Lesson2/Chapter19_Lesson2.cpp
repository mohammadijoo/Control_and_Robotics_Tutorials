#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using std::placeholders::_1;
using ImuMsg = sensor_msgs::msg::Imu;

class ImuLoggerNode : public rclcpp::Node {
public:
  ImuLoggerNode()
  : Node("imu_logger_node"), outfile_("imu_log.csv", std::ios::out)
  {
    outfile_ << "t,ax,ay,az" << std::endl;
    sub_ = this->create_subscription<ImuMsg>(
      "/imu/data", 50, std::bind(&ImuLoggerNode::callback, this, _1));
  }

  ~ImuLoggerNode() {
    outfile_.close();
  }

private:
  void callback(const ImuMsg::SharedPtr msg) {
    double t = static_cast<double>(msg->header.stamp.sec) +
               1e-9 * static_cast<double>(msg->header.stamp.nanosec);
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    outfile_ << t << "," << ax << "," << ay << "," << az << std::endl;
  }

  rclcpp::Subscription<ImuMsg>::SharedPtr sub_;
  std::ofstream outfile_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuLoggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
      
