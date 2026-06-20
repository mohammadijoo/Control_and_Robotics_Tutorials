#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>

using Vector = Eigen::Vector2d;
using Matrix = Eigen::Matrix2d;

struct LinSystem {
  void operator()(const Vector &x, Vector &dxdt, const double t) const {
    Matrix A;
    A << 0.0, 1.0,
         -2.0 - 0.5*std::sin(t), -0.4;
    Vector b;
    b << 0.0, std::cos(2.0*t);
    dxdt = A * x + b;
  }
};

int main() {
  using namespace boost::numeric::odeint;

  Vector x;
  x << 1.0, 0.0;

  double t0 = 0.0, T = 8.0, dt = 1e-3;

  auto obs = [](const Vector &x, double t) {
    if (std::fmod(t, 1.0) < 1e-12) {
      std::cout << "t=" << t << "  x=[" << x(0) << ", " << x(1) << "]\n";
    }
  };

  runge_kutta_dopri5<Vector, double, Vector, double, vector_space_algebra> stepper;
  integrate_const(stepper, LinSystem{}, x, t0, T, dt, obs);

  return 0;
}
      
