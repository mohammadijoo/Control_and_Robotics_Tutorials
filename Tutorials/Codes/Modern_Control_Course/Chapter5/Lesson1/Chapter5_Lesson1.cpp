#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <functional>

struct StateSpace {
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd D;
};

StateSpace nthOrderOdeToSS(const std::vector<double>& a, double b) {
  const int n = static_cast<int>(a.size());
  StateSpace ss;
  ss.A = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n - 1; ++i) ss.A(i, i + 1) = 1.0;

  for (int j = 0; j < n; ++j) ss.A(n - 1, j) = -a[j];

  ss.B = Eigen::MatrixXd::Zero(n, 1);
  ss.B(n - 1, 0) = b;

  ss.C = Eigen::MatrixXd::Zero(1, n);
  ss.C(0, 0) = 1.0;

  ss.D = Eigen::MatrixXd::Zero(1, 1);
  return ss;
}

Eigen::VectorXd rk4Step(
  const std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)>& f,
  double t, const Eigen::VectorXd& x, double h
) {
  Eigen::VectorXd k1 = f(t, x);
  Eigen::VectorXd k2 = f(t + 0.5*h, x + 0.5*h*k1);
  Eigen::VectorXd k3 = f(t + 0.5*h, x + 0.5*h*k2);
  Eigen::VectorXd k4 = f(t + h, x + h*k3);
  return x + (h/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

int main() {
  // Example: y''' + 3 y'' + 3 y' + 1 y = 2 u
  std::vector<double> a = {1.0, 3.0, 3.0}; // [a0, a1, a2]
  double b = 2.0;
  StateSpace ss = nthOrderOdeToSS(a, b);

  auto u = [](double t) { return (t >= 0.0) ? 1.0 : 0.0; };

  auto f = [&ss, &u](double t, const Eigen::VectorXd& x) {
    return ss.A * x + ss.B * u(t);
  };

  double t0 = 0.0, tf = 10.0, h = 1e-3;
  int N = static_cast<int>((tf - t0) / h);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(3); // [y, y', y'']
  double t = t0;

  for (int k = 0; k < N; ++k) {
    x = rk4Step(f, t, x, h);
    t += h;
  }

  double y = (ss.C * x)(0, 0);
  std::cout << "Final y(tf) = " << y << std::endl;
  return 0;
}
