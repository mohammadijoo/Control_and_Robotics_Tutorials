#include <iostream>
#include <vector>
#include <Eigen/Dense>

int main() {
  // Physical parameters
  const double m = 1.0, b = 0.4, k = 4.0;

  // A, B matrices for xdot = A x + B u
  Eigen::Matrix2d A;
  A << 0.0, 1.0,
       -k/m, -b/m;

  Eigen::Vector2d B;
  B << 0.0, 1.0/m;

  // Output y = C x (position)
  Eigen::RowVector2d C;
  C << 1.0, 0.0;

  // Simple explicit Euler integration (educational; small dt needed)
  double dt = 1e-3;
  double T  = 10.0;
  int N = static_cast<int>(T / dt);

  Eigen::Vector2d x;
  x << 0.0, 0.0; // q(0), qdot(0)

  auto u = [](double /*t*/) { return 1.0; }; // unit step

  for (int i = 0; i < N; ++i) {
    double t = i * dt;
    Eigen::Vector2d xdot = A * x + B * u(t);
    x = x + dt * xdot;

    if (i % 2000 == 0) {
      double y = (C * x)(0,0);
      std::cout << "t=" << t << "  q=" << x(0) << "  qdot=" << x(1)
                << "  y=" << y << std::endl;
    }
  }

  std::cout << "Final state x=" << x.transpose() << std::endl;
  return 0;
}
      
