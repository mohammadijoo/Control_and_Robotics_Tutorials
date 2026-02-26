// Chapter4_Lesson3.cpp
// Simple dynamic model for a ground robot: velocity-lag unicycle with RK4 integration.
// Dependencies (recommended): Eigen (header-only). If Eigen is unavailable, replace VectorXd with std::array.
// Build example (Linux):
//   g++ -O2 -std=c++17 Chapter4_Lesson3.cpp -I /usr/include/eigen3 -o amr_dyn

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

struct Params {
  double T_v = 0.30;
  double T_w = 0.25;
};

static double v_ref(double t) { return (t >= 0.5) ? 1.0 : 0.0; }
static double w_ref(double t) { return (t >= 2.0) ? 0.7 : 0.0; }

Eigen::VectorXd f(double t, const Eigen::VectorXd& x, const Params& p) {
  // x = [px, py, theta, v, w]
  Eigen::VectorXd dx(5);
  const double px = x(0), py = x(1), th = x(2), v = x(3), w = x(4);
  dx(0) = v * std::cos(th);
  dx(1) = v * std::sin(th);
  dx(2) = w;
  dx(3) = (v_ref(t) - v) / p.T_v;
  dx(4) = (w_ref(t) - w) / p.T_w;
  return dx;
}

Eigen::VectorXd rk4_step(double t, double h, const Eigen::VectorXd& x, const Params& p) {
  const auto k1 = f(t, x, p);
  const auto k2 = f(t + 0.5*h, x + 0.5*h*k1, p);
  const auto k3 = f(t + 0.5*h, x + 0.5*h*k2, p);
  const auto k4 = f(t + h, x + h*k3, p);
  return x + (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

int main() {
  Params p;
  Eigen::VectorXd x(5);
  x << 0.0, 0.0, 0.0, 0.0, 0.0;

  const double t0 = 0.0, tf = 8.0, h = 0.001;
  double t = t0;

  while (t < tf) {
    x = rk4_step(t, h, x, p);
    t += h;
  }

  std::cout << "Final pose: px=" << x(0) << " py=" << x(1)
            << " theta=" << x(2) << "\n";
  std::cout << "Final body velocities: v=" << x(3) << " w=" << x(4) << "\n";
  return 0;
}
