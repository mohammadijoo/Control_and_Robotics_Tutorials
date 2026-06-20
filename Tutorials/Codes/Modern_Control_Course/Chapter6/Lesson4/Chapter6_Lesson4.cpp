#include <iostream>
#include <vector>
#include <Eigen/Dense>

struct CascadeSystem {
  // x1' = -2 x1 + u
  // x2' = -3 x2 + v,  v = -x1 + u
  // y   = -x2 + v
  Eigen::Vector2d x;

  CascadeSystem() { x.setZero(); }

  void dynamics(double u, Eigen::Vector2d& xdot) const {
    double x1 = x(0);
    double x2 = x(1);
    double v  = -x1 + u;
    xdot(0) = -2.0 * x1 + u;
    xdot(1) = -3.0 * x2 + v;
  }

  double output(double u) const {
    double x1 = x(0);
    double x2 = x(1);
    double v  = -x1 + u;
    double y  = -x2 + v;
    return y;
  }
};

int main() {
  CascadeSystem sys;
  double dt = 0.001;
  double T  = 10.0;
  int N = static_cast<int>(T / dt);

  for (int k = 0; k <= N; ++k) {
    double t = k * dt;
    double u = 1.0; // step input

    // RK4
    Eigen::Vector2d k1, k2, k3, k4, xtmp;
    sys.dynamics(u, k1);

    xtmp = sys.x + 0.5 * dt * k1;
    CascadeSystem s2 = sys; s2.x = xtmp;
    s2.dynamics(u, k2);

    xtmp = sys.x + 0.5 * dt * k2;
    CascadeSystem s3 = sys; s3.x = xtmp;
    s3.dynamics(u, k3);

    xtmp = sys.x + dt * k3;
    CascadeSystem s4 = sys; s4.x = xtmp;
    s4.dynamics(u, k4);

    sys.x += (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

    if (k % 1000 == 0) {
      double y = sys.output(u);
      std::cout << "t=" << t
                << "  x1=" << sys.x(0)
                << "  x2=" << sys.x(1)
                << "  y="  << y << std::endl;
    }
  }

  return 0;
}
      
