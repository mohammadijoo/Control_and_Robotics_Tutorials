
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Vector2d;

struct JointObserver2DOF {
  Matrix4d Ad;
  Eigen::Matrix<double,4,2> Bd;
  Eigen::Matrix<double,2,4> C;
  Eigen::Matrix<double,4,2> Ld;

  Vector4d x_hat;

  JointObserver2DOF() {
    // Example numeric values (should be computed to match your robot model)
    Ad.setIdentity();
    Ad(0,1) = 0.001;
    Ad(2,3) = 0.001;
    // ... fill remaining entries with realistic discretization ...

    Bd.setZero();
    Bd(1,0) = 0.005;
    Bd(3,1) = 0.005;

    C.setZero();
    C(0,0) = 1.0;
    C(1,2) = 1.0;

    // Observer gain Ld (example numbers):
    Ld <<
      0.8,  0.0,
      50.0, 0.0,
      0.0,  0.8,
      0.0,  50.0;

    x_hat.setZero();
  }

  void reset(const Vector4d& x0_hat) {
    x_hat = x0_hat;
  }

  void update(const Vector2d& y_meas,
              const Vector2d& u_k) {
    // Discrete-time observer update:
    // x_hat(k+1) = Ad * x_hat(k) + Bd * u(k) + Ld * (y(k) - C * x_hat(k))
    Vector2d y_hat = C * x_hat;
    x_hat = Ad * x_hat + Bd * u_k + Ld * (y_meas - y_hat);
  }

  Vector4d getStateEstimate() const {
    return x_hat;
  }
};

// In a real robot controller (e.g. in a ROS control loop),
// you would instantiate JointObserver2DOF and call update() each cycle,
// using measured joint positions as y_meas and commanded torques as u_k.
