#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <functional>

struct SS {
  Eigen::MatrixXd A;
  Eigen::VectorXd B;
  Eigen::RowVectorXd C;
  double D;
};

SS companion_from_ode(const std::vector<double>& a, double b) {
  const int n = static_cast<int>(a.size());
  SS sys;
  sys.A = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n - 1; ++i) sys.A(i, i + 1) = 1.0;
  for (int j = 0; j < n; ++j) sys.A(n - 1, j) = -a[j];
  sys.B = Eigen::VectorXd::Zero(n);
  sys.B(n - 1) = b;
  sys.C = Eigen::RowVectorXd::Zero(n);
  sys.C(0) = 1.0;
  sys.D = 0.0;
  return sys;
}

Eigen::VectorXd rk4_step(const std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)>& f,
                         double t, const Eigen::VectorXd& x, double h) {
  Eigen::VectorXd k1 = f(t, x);
  Eigen::VectorXd k2 = f(t + 0.5*h, x + 0.5*h*k1);
  Eigen::VectorXd k3 = f(t + 0.5*h, x + 0.5*h*k2);
  Eigen::VectorXd k4 = f(t + h, x + h*k3);
  return x + (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

int main() {
  // Example: y'' + 3 y' + 2 y = u
  SS sys = companion_from_ode({2.0, 3.0}, 1.0);

  auto u_of_t = [](double t) -> double { return (t >= 0.0) ? 1.0 : 0.0; };

  auto f = [&](double t, const Eigen::VectorXd& x) -> Eigen::VectorXd {
    return sys.A * x + sys.B * u_of_t(t);
  };

  double t0 = 0.0, tf = 5.0, h = 1e-3;
  int N = static_cast<int>((tf - t0) / h);
  Eigen::VectorXd x(2);
  x << 0.0, 0.0; // [y(0), y'(0)]
  double t = t0;

  for (int k = 0; k < N; ++k) {
    x = rk4_step(f, t, x, h);
    t += h;
  }

  double y = sys.C * x + sys.D * u_of_t(t);
  std::cout << "Final y(tf) = " << y << std::endl;
  return 0;
}
