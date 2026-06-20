#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd fundamentalMatrixLTI(const MatrixXd& A, double t0, double t) {
  return (A * (t - t0)).exp(); // Phi(t) = expm(A*(t-t0))
}

// Example time-varying A(t)
MatrixXd Afun(double t) {
  MatrixXd A(2,2);
  A << 0.0, 1.0,
       -(2.0 + 0.1*t), -3.0;
  return A;
}

// One RK4 step for Phi_dot = A(t)*Phi
MatrixXd rk4StepPhi(double t, double h, const MatrixXd& Phi) {
  MatrixXd k1 = Afun(t) * Phi;
  MatrixXd k2 = Afun(t + 0.5*h) * (Phi + 0.5*h*k1);
  MatrixXd k3 = Afun(t + 0.5*h) * (Phi + 0.5*h*k2);
  MatrixXd k4 = Afun(t + h) * (Phi + h*k3);
  return Phi + (h/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

std::vector<MatrixXd> fundamentalMatrixLTV(double t0, const std::vector<double>& tGrid) {
  int n = 2;
  MatrixXd Phi = MatrixXd::Identity(n,n); // Phi(t0)=I
  std::vector<MatrixXd> out;
  out.reserve(tGrid.size());
  out.push_back(Phi);

  double t = t0;
  for (size_t k = 1; k < tGrid.size(); ++k) {
    double tNext = tGrid[k];
    int steps = 50; // simple fixed sub-stepping for stability/accuracy
    double h = (tNext - t) / static_cast<double>(steps);
    for (int s = 0; s < steps; ++s) {
      Phi = rk4StepPhi(t + s*h, h, Phi);
    }
    t = tNext;
    out.push_back(Phi);
  }
  return out;
}

int main() {
  // LTI example
  MatrixXd A(2,2);
  A << 0.0, 1.0,
       -2.0, -3.0;

  double t0 = 0.0;
  double t = 1.25;
  MatrixXd PhiLTI = fundamentalMatrixLTI(A, t0, t);

  VectorXd x0(2);
  x0 << 1.0, 0.0;
  VectorXd x_t = PhiLTI * x0;

  std::cout << "Phi_LTI(t):\n" << PhiLTI << "\n";
  std::cout << "x(t):\n" << x_t << "\n";

  // LTV example
  std::vector<double> tGrid;
  for (int k = 0; k <= 20; ++k) tGrid.push_back(0.1 * k);

  auto PhiGrid = fundamentalMatrixLTV(0.0, tGrid);
  VectorXd x_end = PhiGrid.back() * x0;
  std::cout << "x(2.0) approx:\n" << x_end << "\n";
  return 0;
}
