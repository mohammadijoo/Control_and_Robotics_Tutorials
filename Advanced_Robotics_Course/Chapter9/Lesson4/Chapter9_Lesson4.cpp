#include <Eigen/Dense>
#include <vector>
#include <string>

struct SkeletonStep {
  std::string action; // "move", "grasp", "place", ...
};

struct TAMPProblem {
  int T;
  Eigen::Vector2d q_start;
  Eigen::Vector2d q_goal;
  std::vector<SkeletonStep> skeleton;

  double smoothnessCost(const Eigen::VectorXd& x) const {
    double cost = 0.0;
    int dim = 2;
    for (int k = 0; k < T; ++k) {
      Eigen::Vector2d qk   = x.segment(dim * k, dim);
      Eigen::Vector2d qkp1 = x.segment(dim * (k + 1), dim);
      Eigen::Vector2d v = (qkp1 - qk);
      cost += v.squaredNorm();
    }
    return cost;
  }

  double goalCost(const Eigen::VectorXd& x) const {
    int dim = 2;
    Eigen::Vector2d qT = x.segment(dim * T, dim);
    return (qT - q_goal).squaredNorm();
  }

  double contactCost(const Eigen::VectorXd& x) const {
    // simple mode-dependent penalty (placeholder for real kinematics)
    double cost = 0.0;
    int dim = 2;
    for (int k = 0; k < T; ++k) {
      const auto& step = skeleton[k];
      Eigen::Vector2d qk = x.segment(dim * k, dim);
      if (step.action == "grasp") {
        // encourage qk to be near the object's pose
        Eigen::Vector2d obj(0.5, 0.5);
        cost += (qk - obj).squaredNorm();
      }
    }
    return cost;
  }

  double objective(const Eigen::VectorXd& x) const {
    return smoothnessCost(x) + goalCost(x) + contactCost(x);
  }
};

// An IPOPT or ifopt wrapper would call TAMPProblem::objective(...) and define
// constraint evaluations for joint limits, collisions, dynamics, etc.
// The outer TAMP layer would instantiate multiple TAMPProblem instances with
// different SkeletonStep sequences and choose the best feasible solution.
      
