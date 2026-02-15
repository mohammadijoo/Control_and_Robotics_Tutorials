#include <chrono>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using rclcpp::NodeOptions;

class ReproExperimentNode : public rclcpp::Node {
public:
  explicit ReproExperimentNode(const NodeOptions & options = NodeOptions())
      : Node("repro_experiment_node", options),
        rng_seed_(declare_parameter<int>("global_seed", 1234)),
        num_episodes_(declare_parameter<int>("num_episodes", 50)),
        max_steps_(declare_parameter<int>("max_steps", 200)),
        rng_(static_cast<unsigned int>(rng_seed_)),
        noise_dist_(0.0, 0.01) {

    log_path_ = declare_parameter<std::string>("log_path", "repro_results.csv");
    log_file_.open(log_path_, std::ios::out | std::ios::trunc);
    log_file_ << "episode,total_cost\n";

    command_pub_ = create_publisher<std_msgs::msg::Float64>(
        "joint1/command", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ReproExperimentNode::control_loop, this));
  }

private:
  void control_loop() {
    if (episode_ >= num_episodes_) {
      RCLCPP_INFO(get_logger(), "All episodes finished, shutting down.");
      rclcpp::shutdown();
      return;
    }
    if (step_ == 0) {
      RCLCPP_INFO(get_logger(), "Starting episode %d", episode_);
      episode_cost_ = 0.0;
    }

    double nominal_cmd = scripted_command(step_);
    double noisy_cmd = nominal_cmd + noise_dist_(rng_);

    std_msgs::msg::Float64 msg;
    msg.data = noisy_cmd;
    command_pub_->publish(msg);

    episode_cost_ += std::abs(noisy_cmd);

    ++step_;
    if (step_ >= max_steps_) {
      log_file_ << episode_ << "," << episode_cost_ << "\n";
      ++episode_;
      step_ = 0;
    }
  }

  double scripted_command(int step) const {
    // Placeholder: simple sinusoidal profile
    return 0.5 * std::sin(0.01 * static_cast<double>(step));
  }

  int rng_seed_;
  int num_episodes_;
  int max_steps_;

  int episode_ = 0;
  int step_ = 0;
  double episode_cost_ = 0.0;

  std::mt19937 rng_;
  std::normal_distribution<double> noise_dist_;

  std::string log_path_;
  std::ofstream log_file_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReproExperimentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
      
