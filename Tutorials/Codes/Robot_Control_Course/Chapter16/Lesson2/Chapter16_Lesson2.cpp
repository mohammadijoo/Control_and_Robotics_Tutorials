
#pragma once

#include <Eigen/Dense>
#include <memory>

struct Reference {
  Eigen::VectorXd qd;
  Eigen::VectorXd dqd;
  Eigen::VectorXd ddqd;
};

class RobotModel {
public:
  virtual ~RobotModel() {}
  virtual int dof() const = 0;
  virtual Eigen::MatrixXd M(const Eigen::VectorXd& q) const = 0;
  virtual Eigen::MatrixXd C(const Eigen::VectorXd& q,
                            const Eigen::VectorXd& dq) const = 0;
  virtual Eigen::VectorXd g(const Eigen::VectorXd& q) const = 0;
};

class Controller {
public:
  virtual ~Controller() {}
  virtual Eigen::VectorXd
  computeTau(double t,
             const Eigen::VectorXd& q,
             const Eigen::VectorXd& dq,
             const Reference& ref) = 0;
};

class JointPDController : public Controller {
public:
  JointPDController(const Eigen::VectorXd& kp,
                    const Eigen::VectorXd& kd)
    : Kp(kp.asDiagonal()), Kd(kd.asDiagonal()) {}

  Eigen::VectorXd computeTau(double /*t*/,
                             const Eigen::VectorXd& q,
                             const Eigen::VectorXd& dq,
                             const Reference& ref) override {
    Eigen::VectorXd e  = ref.qd  - q;
    Eigen::VectorXd de = ref.dqd - dq;
    return Kp * e + Kd * de;
  }

private:
  Eigen::MatrixXd Kp, Kd;
};

class ComputedTorqueController : public Controller {
public:
  ComputedTorqueController(std::shared_ptr<RobotModel> model,
                           const Eigen::VectorXd& kp,
                           const Eigen::VectorXd& kd)
    : model_(std::move(model)),
      Kp(kp.asDiagonal()), Kd(kd.asDiagonal()) {}

  Eigen::VectorXd computeTau(double /*t*/,
                             const Eigen::VectorXd& q,
                             const Eigen::VectorXd& dq,
                             const Reference& ref) override {
    Eigen::VectorXd e  = ref.qd  - q;
    Eigen::VectorXd de = ref.dqd - dq;
    Eigen::VectorXd v  = ref.ddqd + Kd * de + Kp * e;

    Eigen::MatrixXd M = model_->M(q);
    Eigen::MatrixXd C = model_->C(q, dq);
    Eigen::VectorXd g = model_->g(q);

    return M * v + C * dq + g;
  }

private:
  std::shared_ptr<RobotModel> model_;
  Eigen::MatrixXd Kp, Kd;
};

class SafetyFilter {
public:
  SafetyFilter(const Eigen::VectorXd& umin,
               const Eigen::VectorXd& umax)
    : umin_(umin), umax_(umax) {}

  Eigen::VectorXd filter(const Eigen::VectorXd& u_nom) const {
    Eigen::VectorXd u = u_nom;
    for (int i = 0; i < u.size(); ++i) {
      if (u[i] < umin_[i]) u[i] = umin_[i];
      if (u[i] > umax_[i]) u[i] = umax_[i];
    }
    return u;
  }

private:
  Eigen::VectorXd umin_, umax_;
};

class Architecture {
public:
  Architecture(std::shared_ptr<Controller> controller,
               std::shared_ptr<SafetyFilter> safety = nullptr)
    : controller_(std::move(controller)), safety_(std::move(safety)) {}

  Eigen::VectorXd computeTau(double t,
                             const Eigen::VectorXd& q,
                             const Eigen::VectorXd& dq,
                             const Reference& ref) {
    Eigen::VectorXd u_nom = controller_->computeTau(t, q, dq, ref);
    if (safety_) {
      return safety_->filter(u_nom);
    }
    return u_nom;
  }

private:
  std::shared_ptr<Controller> controller_;
  std::shared_ptr<SafetyFilter> safety_;
};
