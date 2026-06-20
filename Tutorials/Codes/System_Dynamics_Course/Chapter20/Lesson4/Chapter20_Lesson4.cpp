// Chapter20_Lesson4.cpp
/*
Chapter 20 — Chaos, Complex Dynamics, and Computational Tools
Lesson 4 — Computational Tools

C++ (standard library only) demonstration:
- Lorenz ODE simulated with fixed-step RK4
- Simple Poincaré section detection (x crosses 0 with positive direction)
- Logistic map bifurcation tail data and Lyapunov exponent for the map
- CSV export for plotting in MATLAB/Python/etc.

Build:
  g++ -O2 -std=c++17 Chapter20_Lesson4.cpp -o ch20_l4

Run:
  ./ch20_l4
*/

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

struct LorenzParams {
  double sigma = 10.0;
  double rho   = 28.0;
  double beta  = 8.0 / 3.0;
};

struct State3 {
  double x, y, z;
};

static inline State3 add(const State3& a, const State3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
static inline State3 mul(double s, const State3& a) {
  return {s * a.x, s * a.y, s * a.z};
}

static inline State3 lorenz_rhs(const State3& s, const LorenzParams& p) {
  return {
    p.sigma * (s.y - s.x),
    s.x * (p.rho - s.z) - s.y,
    s.x * s.y - p.beta * s.z
  };
}

static inline State3 rk4_step(const State3& s, double h, const LorenzParams& p) {
  State3 k1 = lorenz_rhs(s, p);
  State3 k2 = lorenz_rhs(add(s, mul(h * 0.5, k1)), p);
  State3 k3 = lorenz_rhs(add(s, mul(h * 0.5, k2)), p);
  State3 k4 = lorenz_rhs(add(s, mul(h, k3)), p);

  State3 incr = mul(h / 6.0, add(add(k1, mul(2.0, k2)), add(mul(2.0, k3), k4)));
  return add(s, incr);
}

static inline double logistic_next(double r, double x) {
  return r * x * (1.0 - x);
}

static double logistic_lyapunov(double r, double x0, int n_transient, int n) {
  double x = x0;
  for (int i = 0; i < n_transient; ++i) x = logistic_next(r, x);
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    x = logistic_next(r, x);
    double d = std::abs(r * (1.0 - 2.0 * x));
    if (d < 1e-300) d = 1e-300;
    sum += std::log(d);
  }
  return sum / static_cast<double>(n);
}

int main() {
  const std::string outDir = "outputs_ch20_l4_cpp";

  // --- Lorenz simulation ---
  LorenzParams p;
  State3 s{1.0, 1.0, 1.0};
  double t = 0.0;
  const double T = 40.0;
  const double h = 0.001; // fixed step for simplicity

  std::ofstream traj(outDir + "_lorenz_traj.csv");
  traj << "t,x,y,z\n";

  std::ofstream poinc(outDir + "_lorenz_poincare_x0.csv");
  poinc << "x,y,z\n";

  double prev_x = s.x;
  for (int k = 0; t <= T; ++k) {
    traj << t << "," << s.x << "," << s.y << "," << s.z << "\n";

    // detect crossing x: negative -> positive
    if (prev_x < 0.0 && s.x >= 0.0) {
      poinc << s.x << "," << s.y << "," << s.z << "\n";
    }
    prev_x = s.x;

    s = rk4_step(s, h, p);
    t += h;
  }

  std::cout << "Wrote Lorenz CSV files: " << outDir << "_lorenz_*.csv\n";

  // --- Logistic map: bifurcation tail data ---
  std::ofstream bif(outDir + "_logistic_bifurcation.csv");
  bif << "r,x\n";
  const double r_min = 2.5;
  const double r_max = 4.0;
  const int n_r = 400;
  const int n_trans = 800;
  const int n_keep = 120;

  for (int i = 0; i < n_r; ++i) {
    double r = r_min + (r_max - r_min) * (static_cast<double>(i) / (n_r - 1));
    double x = 0.2;
    for (int k = 0; k < n_trans; ++k) x = logistic_next(r, x);
    for (int k = 0; k < n_keep; ++k) {
      x = logistic_next(r, x);
      bif << r << "," << x << "\n";
    }
  }

  // --- Logistic map: Lyapunov curve ---
  std::ofstream ly(outDir + "_logistic_lyapunov.csv");
  ly << "r,lambda\n";
  const int n_r2 = 200;
  for (int i = 0; i < n_r2; ++i) {
    double r = 2.8 + (4.0 - 2.8) * (static_cast<double>(i) / (n_r2 - 1));
    double lam = logistic_lyapunov(r, 0.2, 1000, 4000);
    ly << r << "," << lam << "\n";
  }

  std::cout << "Wrote logistic CSV files: " << outDir << "_logistic_*.csv\n";
  return 0;
}
