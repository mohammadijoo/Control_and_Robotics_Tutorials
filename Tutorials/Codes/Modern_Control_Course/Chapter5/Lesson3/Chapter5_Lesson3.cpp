#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

int main() {
  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  // Matrices from Section 5
  MatrixXd A(2,2), B(2,2), C(2,2), D(2,2);
  A << 0.0, 1.0,
      -2.0, -3.0;
  B << 0.0, 0.0,
       4.0, -1.0;
  C << 1.0, 0.0,
       0.0, 1.0;
  D << 0.0, 0.0,
       0.0, 0.5;

  double T = 10.0;
  double dt = 0.005;
  int N = static_cast<int>(T / dt) + 1;

  VectorXd x = VectorXd::Zero(2);

  for (int k = 0; k < N; ++k) {
    double t = k * dt;

    // Inputs u1(t)=1, u2(t)=exp(-0.7 t)
    VectorXd u(2);
    u(0) = 1.0;
    u(1) = std::exp(-0.7 * t);

    // Output
    VectorXd y = C * x + D * u;

    // Print a few samples
    if (k % 400 == 0) {
      std::cout << "t=" << t
                << "  y1=" << y(0)
                << "  y2=" << y(1)
                << "  x1=" << x(0)
                << "  x2=" << x(1)
                << std::endl;
    }

    // Euler step
    VectorXd xdot = A * x + B * u;
    x = x + dt * xdot;
  }

  return 0;
}
      
