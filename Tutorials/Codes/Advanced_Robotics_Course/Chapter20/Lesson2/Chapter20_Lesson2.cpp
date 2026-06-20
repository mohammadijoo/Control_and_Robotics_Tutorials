#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"

struct MethodChoice {
  enum class PlannerKind { RRT_STAR, TRAJ_OPT };
  enum class EstimatorKind { EKF, PARTICLE };
  PlannerKind planner;
  EstimatorKind estimator;
  bool use_learned_policy;
};

class AutonomyNode : public rclcpp::Node {
public:
  explicit AutonomyNode(const MethodChoice & choice)
  : Node("autonomy_node"), choice_(choice)
  {
    // Initialize publishers, subscribers, timers
  }

private:
  MethodChoice choice_;
  // Perception, estimator, planner, controller modules as members

  void sensorCallback(const std::shared_ptr<const SensorMsg> msg)
  {
    // Process perception, estimation, planning, control, monitoring
  }
};
      
