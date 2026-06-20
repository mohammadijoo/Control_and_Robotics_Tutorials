#include <cmath>
#include <array>

// Simple pendulum dynamics: x = [q, qd]
struct PendulumParams {
  double m;
  double l;
  double g;
};

// User-specified torque profile
double torque(double t, double q, double qd) {
  // Example: simple PD control about q = 0
  double Kp = 5.0;
  double Kd = 1.0;
  return -Kp * q - Kd * qd;
}

// Right-hand side of the ODE: xdot = f(t, x)
std::array<double, 2> pendulum_rhs(double t,
                                    const std::array<double, 2>& x,
                                    const PendulumParams& p) {
  double q  = x[0];
  double qd = x[1];

  double tau = torque(t, q, qd);

  double denom = p.m * p.l * p.l;
  double qdd   = (tau - p.m * p.g * p.l * std::sin(q)) / denom;

  return {qd, qdd};
}
      
