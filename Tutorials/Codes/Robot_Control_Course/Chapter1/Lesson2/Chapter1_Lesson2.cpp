
#include <Eigen/Dense>
#include <functional>

struct Joint1DOFStateSpace {
  double J;
  double b;
  double k;

  // x = [q; q_dot]
  Eigen::Vector2d f(double t,
                    const Eigen::Vector2d& x,
                    const std::function<double(double)>& u) const {
    double q = x(0);
    double qdot = x(1);
    double tau = u(t);

    double qddot = (tau - b * qdot - k * q) / J;
    Eigen::Vector2d xdot;
    xdot << qdot, qddot;
    return xdot;
  }

  Eigen::Matrix2d A() const {
    Eigen::Matrix2d A;
    A << 0.0, 1.0,
           -k / J, -b / J;
    return A;
  }

  Eigen::Vector2d B() const {
    Eigen::Vector2d B;
    B << 0.0, 1.0 / J;
    return B;
  }

  Eigen::RowVector2d C() const {
    Eigen::RowVector2d C;
    C << 1.0, 0.0;
    return C;
  }

  double D() const { return 0.0; }
};

// Example usage (integration would be done by your ODE integrator of choice)
