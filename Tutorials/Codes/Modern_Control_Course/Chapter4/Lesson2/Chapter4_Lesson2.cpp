#include <Eigen/Dense>
#include <iostream>
#include <vector>

struct LTISystem {
  Eigen::MatrixXd A, B, C, D;

  Eigen::VectorXd xdot(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
    return A * x + B * u;
  }

  Eigen::VectorXd y(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
    return C * x + D * u;
  }
};

// Classic RK4 step for xdot = f(x,t)
Eigen::VectorXd rk4_step(
    const LTISystem& sys,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    double h)
{
  Eigen::VectorXd k1 = sys.xdot(x, u);
  Eigen::VectorXd k2 = sys.xdot(x + 0.5*h*k1, u);
  Eigen::VectorXd k3 = sys.xdot(x + 0.5*h*k2, u);
  Eigen::VectorXd k4 = sys.xdot(x + h*k3, u);
  return x + (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

int main() {
  LTISystem sys;
  sys.A.resize(2,2);
  sys.B.resize(2,1);
  sys.C.resize(1,2);
  sys.D.resize(1,1);

  sys.A << 0.0, 1.0,
          -2.0, -3.0;
  sys.B << 0.0,
           1.0;
  sys.C << 1.0, 0.0;
  sys.D << 0.0;

  double t0 = 0.0, tf = 5.0, h = 1e-3;
  int N = static_cast<int>((tf - t0)/h);

  Eigen::VectorXd x(2); x << 0.5, 0.0;
  Eigen::VectorXd u(1); u << 1.0; // step input

  for (int k = 0; k < N; ++k) {
    x = rk4_step(sys, x, u, h);
  }

  Eigen::VectorXd y = sys.y(x, u);
  std::cout << "x(tf) =\n" << x << "\n";
  std::cout << "y(tf) =\n" << y << "\n";
  return 0;
}
      
