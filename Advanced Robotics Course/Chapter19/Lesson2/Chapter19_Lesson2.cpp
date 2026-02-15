#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <unordered_map>
#include <string>

// A tiny linear generalist policy over concatenated features.
class GeneralistPolicy {
public:
  GeneralistPolicy(int state_dim, int context_dim, int action_dim)
      : W_(action_dim, state_dim + context_dim),
        b_(action_dim) {
    W_.setZero();
    b_.setZero();
  }

  Eigen::VectorXd act(const Eigen::VectorXd &x,
                      const Eigen::VectorXd &c) const {
    Eigen::VectorXd z(x.size() + c.size());
    z << x, c;
    return W_ * z + b_;
  }

  // In a real system, parameters would be loaded from disk.
  void loadDummyParameters() {
    W_.setRandom();
    b_.setZero();
  }

private:
  Eigen::MatrixXd W_;
  Eigen::VectorXd b_;
};

class GeneralistNode : public rclcpp::Node {
public:
  GeneralistNode()
      : Node("generalist_policy_node"),
        policy_(STATE_DIM, CONTEXT_DIM, ACTION_DIM) {
    policy_.loadDummyParameters();
    // TODO: subscribe to observation topics and task descriptor,
    // publish to controller command topic.
  }

private:
  static constexpr int STATE_DIM = 12;   // e.g. joint positions/velocities
  static constexpr int CONTEXT_DIM = 8;  // e.g. task embedding
  static constexpr int ACTION_DIM = 6;   // e.g. joint torques

  GeneralistPolicy policy_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GeneralistNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
      
