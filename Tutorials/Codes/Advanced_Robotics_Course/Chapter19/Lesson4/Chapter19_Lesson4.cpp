#pragma once
#include "Eigen/Dense"

class WorldModel {
public:
    using State = Eigen::VectorXd;
    using Action = Eigen::VectorXd;

    virtual ~WorldModel() {}

    // Predict mean next state from current state and action
    virtual State predictNextState(const State& z,
                                   const Action& u) const = 0;

    // Optionally, return covariance of prediction for uncertainty-aware MPC
    virtual Eigen::MatrixXd predictCovariance(const State& z,
                                              const Action& u) const = 0;
};

// A simple linear world model: z_{t+1} = A z_t + B u_t
class LinearWorldModel : public WorldModel {
public:
    LinearWorldModel(int state_dim, int action_dim)
        : A_(State::Zero(state_dim)),
          B_(Eigen::MatrixXd::Zero(state_dim, action_dim)) {}

    void setA(const Eigen::MatrixXd& A) { A_ = A; }
    void setB(const Eigen::MatrixXd& B) { B_ = B; }

    State predictNextState(const State& z,
                           const Action& u) const override {
        return A_ * z + B_ * u;
    }

    Eigen::MatrixXd predictCovariance(const State&,
                                      const Action&) const override {
        // For a deterministic linear model, covariance is zero.
        return Eigen::MatrixXd::Zero(A_.rows(), A_.rows());
    }

private:
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
};
      
