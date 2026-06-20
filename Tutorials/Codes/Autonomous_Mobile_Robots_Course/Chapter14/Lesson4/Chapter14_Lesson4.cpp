/* 
Chapter14_Lesson4.cpp
Recovery Behaviors and Fault Handling — ROS 2 / Nav2-oriented supervisor (example)

Build example (adapt your ROS 2 distro):
  add_executable(recovery_supervisor src/Chapter14_Lesson4.cpp)
  ament_target_dependencies(recovery_supervisor rclcpp geometry_msgs nav_msgs nav2_msgs)
*/

#include <chrono>
#include <cmath>
#include <deque>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

using namespace std::chrono_literals;

struct MonitorParams {
  double window_s = 10.0;
  double min_progress_m = 0.25;

  double osc_window_s = 6.0;
  int osc_sign_flips = 6;
  double v_eps = 0.03;

  double cov_trace_max = 0.35;

  double spin_time_s = 3.0;
  double spin_wz = 0.8;
  double backup_time_s = 2.0;
  double backup_vx = -0.12;

  int max_recovery_attempts = 3;

  std::string cmd_vel_topic = "/cmd_vel";
  std::string odom_topic = "/odom";
  std::string amcl_topic = "/amcl_pose";
  std::string local_clear_srv = "/local_costmap/clear_entirely_local_costmap";
  std::string global_clear_srv = "/global_costmap/clear_entirely_global_costmap";
};

class RecoverySupervisor : public rclcpp::Node {
public:
  RecoverySupervisor() : Node("recovery_supervisor_cpp") {
    p_ = MonitorParams{};

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(p_.cmd_vel_topic, 10);

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      p_.odom_topic, rclcpp::QoS(10).best_effort(),
      std::bind(&RecoverySupervisor::on_odom, this, std::placeholders::_1));

    sub_amcl_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      p_.amcl_topic, 10,
      std::bind(&RecoverySupervisor::on_amcl, this, std::placeholders::_1));

    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      p_.cmd_vel_topic, 10,
      std::bind(&RecoverySupervisor::on_cmd, this, std::placeholders::_1));

    cli_local_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(p_.local_clear_srv);
    cli_global_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(p_.global_clear_srv);

    goal_set_ = true;
    goal_x_ = 5.0;
    goal_y_ = 0.0;

    timer_ = this->create_wall_timer(200ms, std::bind(&RecoverySupervisor::tick, this));
    RCLCPP_INFO(this->get_logger(), "RecoverySupervisor C++ started (demo goal set to (5,0)).");
  }

private:
  // history entries
  using DistEntry = std::pair<double, double>; // (t, dist)
  struct CmdEntry { double t; double vx; double wz; };

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_x_ = msg->pose.pose.position.x;
    last_y_ = msg->pose.pose.position.y;
    have_pose_ = true;

    if (goal_set_) {
      double d = std::hypot(goal_x_ - last_x_, goal_y_ - last_y_);
      double t = now_s();
      progress_hist_.push_back({t, d});
      trim_progress_();
    }
  }

  void on_amcl(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // covariance is 6x6 row-major in array[36]
    const auto & c = msg->pose.covariance;
    // trace of (x,y,yaw) approx indices: (0,0), (1,1), (5,5)
    last_cov_trace_ = c[0] + c[7] + c[35];
    have_cov_ = true;
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double t = now_s();
    cmd_hist_.push_back({t, msg->linear.x, msg->angular.z});
    trim_cmd_();
  }

  double now_s() const {
    return this->get_clock()->now().nanoseconds() * 1e-9;
  }

  void trim_progress_() {
    if (progress_hist_.empty()) return;
    double t_latest = progress_hist_.back().first;
    double t_min = t_latest - p_.window_s;
    while (!progress_hist_.empty() && progress_hist_.front().first < t_min) {
      progress_hist_.pop_front();
    }
  }

  void trim_cmd_() {
    if (cmd_hist_.empty()) return;
    double t_latest = cmd_hist_.back().t;
    double t_min = t_latest - p_.osc_window_s;
    while (!cmd_hist_.empty() && cmd_hist_.front().t < t_min) {
      cmd_hist_.pop_front();
    }
  }

  bool progress_ok_() const {
    if (!goal_set_ || progress_hist_.size() < 2) return true;
    double d0 = progress_hist_.front().second;
    double d1 = progress_hist_.back().second;
    return (d0 - d1) >= p_.min_progress_m;
  }

  int sgn_eps_(double v) const {
    if (std::fabs(v) < p_.v_eps) return 0;
    return (v > 0.0) ? 1 : -1;
  }

  bool oscillating_() const {
    if (cmd_hist_.size() < 3) return false;
    int flips = 0;
    for (size_t i = 1; i < cmd_hist_.size(); ++i) {
      int s_prev = sgn_eps_(cmd_hist_[i-1].vx);
      int s_now  = sgn_eps_(cmd_hist_[i].vx);
      if (s_prev != 0 && s_now != 0 && s_prev != s_now) flips++;
    }
    for (size_t i = 1; i < cmd_hist_.size(); ++i) {
      int s_prev = sgn_eps_(cmd_hist_[i-1].wz);
      int s_now  = sgn_eps_(cmd_hist_[i].wz);
      if (s_prev != 0 && s_now != 0 && s_prev != s_now) flips++;
    }
    return flips >= p_.osc_sign_flips;
  }

  bool localization_bad_() const {
    if (!have_cov_) return false;
    return last_cov_trace_ > p_.cov_trace_max;
  }

  void publish_for_(double vx, double wz, double duration_s) {
    auto t_end = this->get_clock()->now() + rclcpp::Duration::from_seconds(duration_s);
    rclcpp::Rate rate(20.0);
    geometry_msgs::msg::Twist cmd;
    while (rclcpp::ok() && this->get_clock()->now() < t_end) {
      cmd.linear.x = vx;
      cmd.angular.z = wz;
      pub_cmd_->publish(cmd);
      rate.sleep();
    }
    cmd.linear.x = 0.0; cmd.angular.z = 0.0;
    pub_cmd_->publish(cmd);
  }

  void call_clear_costmaps_() {
    for (auto & cli : {cli_local_, cli_global_}) {
      if (!cli->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Costmap clear service not ready: %s", cli->get_service_name());
        continue;
      }
      auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      auto fut = cli->async_send_request(req);
      auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut, 2s);
      RCLCPP_INFO(this->get_logger(), "Clear costmap %s", (ret == rclcpp::FutureReturnCode::SUCCESS) ? "OK" : "TIMEOUT/FAIL");
    }
  }

  void do_recovery_(const std::string & reason) {
    in_recovery_ = true;
    recovery_attempts_++;

    RCLCPP_WARN(this->get_logger(), "RECOVERY #%d reason=%s", recovery_attempts_, reason.c_str());

    call_clear_costmaps_();
    publish_for_(0.0, p_.spin_wz, p_.spin_time_s);
    publish_for_(p_.backup_vx, 0.0, p_.backup_time_s);

    in_recovery_ = false;
  }

  void safe_stop_(const std::string & reason) {
    RCLCPP_ERROR(this->get_logger(), "SAFE STOP: %s", reason.c_str());
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    for (int i = 0; i < 10; ++i) {
      pub_cmd_->publish(cmd);
      rclcpp::sleep_for(50ms);
    }
  }

  void tick() {
    if (in_recovery_) return;

    if (recovery_attempts_ >= p_.max_recovery_attempts) {
      safe_stop_("exceeded max recovery attempts");
      rclcpp::shutdown();
      return;
    }

    if (localization_bad_()) {
      do_recovery_("localization covariance too high");
      return;
    }

    if (goal_set_) {
      if (!progress_ok_() || oscillating_()) {
        do_recovery_(!progress_ok_() ? "no progress" : "oscillation");
        return;
      }
    }
  }

private:
  MonitorParams p_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;

  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr cli_local_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr cli_global_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool have_pose_ = false;
  double last_x_ = 0.0, last_y_ = 0.0;

  bool have_cov_ = false;
  double last_cov_trace_ = 0.0;

  bool goal_set_ = false;
  double goal_x_ = 0.0, goal_y_ = 0.0;

  std::deque<DistEntry> progress_hist_;
  std::deque<CmdEntry> cmd_hist_;

  int recovery_attempts_ = 0;
  bool in_recovery_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RecoverySupervisor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
