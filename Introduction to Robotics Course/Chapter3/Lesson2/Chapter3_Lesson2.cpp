#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix2d;
using Eigen::Vector2d;

struct State {
  double x, xdot, q, qdot;
};

int main() {
  // Parameters
  const double I=0.2, M=1.0, m=0.5, ell=0.6, g=9.81;
  const double Kp=15.0, Kd=3.0;

  double dt = 0.001;
  State s{0.0, 0.0, 0.5, 0.0};

  for(int i=0; i<5000; ++i){
    double tau = -Kp*s.q - Kd*s.qdot;
    double ub  = 0.0;

    // Mass matrix
    double M11 = M + m;
    double M12 = m*ell*std::cos(s.q);
    double M22 = I + m*ell*ell;

    Matrix2d Mass;
    Mass << M11, M12,
            M12, M22;

    // Bias terms
    double h1 = -m*ell*std::sin(s.q)*s.qdot*s.qdot;
    double h2 =  m*g*ell*std::sin(s.q);

    Vector2d rhs;
    rhs << ub - h1, tau - h2;

    Vector2d acc = Mass.lu().solve(rhs);
    double xddot = acc(0);
    double qddot = acc(1);

    // Euler integration
    s.x    += dt*s.xdot;
    s.xdot += dt*xddot;
    s.q    += dt*s.qdot;
    s.qdot += dt*qddot;
  }

  std::cout << "Final q=" << s.q << " Final x=" << s.x << std::endl;
  return 0;
}
      