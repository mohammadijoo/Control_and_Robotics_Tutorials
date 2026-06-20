// Chapter14_Lesson1.cpp
// ROS 2 layered navigation safety filter (local->reactive) skeleton.
// Build conceptually with a ROS 2 workspace (ament_cmake).
//
// Author: Abolfazl Mohammadijoo (course material)
// License: MIT (see repository)

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

static double wrap_to_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double yaw_from_quat(double w, double x, double y, double z) {
  // yaw (z-axis rotation)
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}

static std::pair<double,double> safety_filter_scan(
    const std::pair<double,double>& u_nom,
    double min_range,
    double d_stop,
    double d_slow)
{
  double v = u_nom.first;
  double w = u_nom.second;

  if (min_range <= d_stop) {
    return {0.0, 0.0};
  }
  if (min_range <= d_slow) {
    double s = (min_range - d_stop) / (d_slow - d_stop + 1e-9);
    s = std::clamp(s, 0.0, 1.0);
    return {s * v, s * w};
  }
  return {v, w};
}

class LayeredNavFilter : public rclcpp::Node {
public:
  LayeredNavFilter() : Node("layered_nav_filter_cpp") {
    this->declare_parameter<double>("v_ref", 0.4);
    this->declare_parameter<double>("k_yaw", 1.8);
    this->declare_parameter<double>("d_stop", 0.35);
    this->declare_parameter<double>("d_slow", 0.9);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg){ cb_odom(*msg); });

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 50,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){ cb_scan(*msg); });

    // Optional: if you want to forward a global path, subscribe here.
    // sub_path_ = ...

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this](){ on_timer(); });
  }

private:
  void cb_odom(const nav_msgs::msg::Odometry& msg) {
    pose_.x = msg.pose.pose.position.x;
    pose_.y = msg.pose.pose.position.y;
    const auto& q = msg.pose.pose.orientation;
    pose_.yaw = yaw_from_quat(q.w, q.x, q.y, q.z);
  }

  void cb_scan(const sensor_msgs::msg::LaserScan& msg) {
    double mr = std::numeric_limits<double>::infinity();
    for (const auto r : msg.ranges) {
      if (std::isfinite(r) && r > 0.0) mr = std::min(mr, (double)r);
    }
    min_range_ = mr;
  }

  std::pair<double,double> nominal_local_cmd() const {
    // Very simple local nominal command (stand-in for a real controller):
    const double v_ref = this->get_parameter("v_ref").as_double();
    const double k_yaw = this->get_parameter("k_yaw").as_double();

    // Drive forward; damp yaw rate toward zero (assumes global layer keeps heading reasonable).
    const double v = v_ref;
    const double w = -k_yaw * wrap_to_pi(pose_.yaw - 0.0);
    return {v, w};
  }

  void on_timer() {
    const double d_stop = this->get_parameter("d_stop").as_double();
    const double d_slow = this->get_parameter("d_slow").as_double();

    auto u_nom = nominal_local_cmd();
    auto u_safe = safety_filter_scan(u_nom, min_range_, d_stop, d_slow);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = u_safe.first;
    cmd.angular.z = u_safe.second;
    pub_cmd_->publish(cmd);
  }

private:
  Pose2D pose_;
  double min_range_{std::numeric_limits<double>::infinity()};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LayeredNavFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
