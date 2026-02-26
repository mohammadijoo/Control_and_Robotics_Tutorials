
#include <iostream>
#include <cmath>

struct SecondOrderSpecs {
  double zeta;
  double omega_n;
};

struct StepMetrics {
  double Mp;
  double Ts;
};

StepMetrics simulateAndMetrics(const SecondOrderSpecs& specs,
                               double dt, double t_final) {
  double zeta = specs.zeta;
  double wn = specs.omega_n;

  // State x = [y; ydot]
  double y = 0.0;
  double ydot = 0.0;
  double r = 1.0;

  double y_inf = 1.0;
  double y_max = y;
  double Ts = t_final;
  double tol = 0.02 * std::fabs(y_inf);
  bool settled = false;

  int n_steps = static_cast<int>(t_final / dt);
  for (int k = 0; k < n_steps; ++k) {
    double t = k * dt;
    // Dynamics: yddot + 2 zeta wn ydot + wn^2 y = wn^2 r
    double yddot = -2.0 * zeta * wn * ydot - wn * wn * y + wn * wn * r;

    // Forward Euler integration
    ydot += dt * yddot;
    y += dt * ydot;

    if (y > y_max) y_max = y;

    // Check settling (2 percent band)
    if (!settled) {
      if (std::fabs(y - y_inf) <= tol) {
        // Tentatively mark time, but require it to stay within band
        bool ok = true;
        double y_temp = y;
        double ydot_temp = ydot;
        for (int j = k + 1; j < n_steps; ++j) {
          double t2 = j * dt;
          double yddot2 = -2.0 * zeta * wn * ydot_temp
                          - wn * wn * y_temp + wn * wn * r;
          ydot_temp += dt * yddot2;
          y_temp += dt * ydot_temp;
          if (std::fabs(y_temp - y_inf) > tol) {
            ok = false;
            break;
          }
        }
        if (ok) {
          Ts = t;
          settled = true;
        }
      }
    }
  }

  double Mp = 0.0;
  if (y_inf != 0.0) {
    Mp = 100.0 * (y_max - y_inf) / std::fabs(y_inf);
    if (Mp < 0.0) Mp = 0.0;
  }

  StepMetrics metrics;
  metrics.Mp = Mp;
  metrics.Ts = Ts;
  return metrics;
}

int main() {
  SecondOrderSpecs specs{0.7, 8.0};
  StepMetrics m = simulateAndMetrics(specs, 1e-3, 5.0);
  std::cout << "Mp = " << m.Mp << " percent\n";
  std::cout << "Ts = " << m.Ts << " s\n";
  return 0;
}
