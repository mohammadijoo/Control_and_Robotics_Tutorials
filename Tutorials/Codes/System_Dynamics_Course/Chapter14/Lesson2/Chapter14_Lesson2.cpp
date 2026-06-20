/*
Chapter14_Lesson2.cpp
Equilibrium points, local classification, and trajectory simulation (RK4) for a 2D autonomous system.

Build (example):
  g++ -O2 -std=c++17 Chapter14_Lesson2.cpp -o Chapter14_Lesson2

Run:
  ./Chapter14_Lesson2
Outputs:
  trajectories.csv   (columns: traj_id,t,x,y)
*/

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

struct Vec2 {
  double x;
  double y;
};

struct LinClass {
  std::string kind;
  double tr;
  double det;
  double disc;
};

static Vec2 F(double /*t*/, const Vec2& z, double a) {
  // Example system:
  // x' = x - x^3 - y
  // y' = x + a y
  return Vec2{z.x - z.x*z.x*z.x - z.y, z.x + a*z.y};
}

static void jacobian(const Vec2& z, double a, double J[2][2]) {
  // Analytical Jacobian for the example system
  // f_x = 1 - 3 x^2, f_y = -1
  // g_x = 1,        g_y = a
  J[0][0] = 1.0 - 3.0*z.x*z.x;
  J[0][1] = -1.0;
  J[1][0] = 1.0;
  J[1][1] = a;
}

static LinClass classify2x2(const double J[2][2], double eps=1e-12) {
  const double tr  = J[0][0] + J[1][1];
  const double det = J[0][0]*J[1][1] - J[0][1]*J[1][0];
  const double disc = tr*tr - 4.0*det;

  std::string kind;
  if (det < -eps) {
    kind = "saddle (hyperbolic)";
  } else if (std::abs(det) <= eps) {
    kind = "degenerate (det ~ 0)";
  } else {
    if (disc > eps) {
      if (tr < -eps) kind = "stable node";
      else if (tr > eps) kind = "unstable node";
      else kind = "improper/star node";
    } else if (disc < -eps) {
      if (tr < -eps) kind = "stable spiral (focus)";
      else if (tr > eps) kind = "unstable spiral (focus)";
      else kind = "center (linear); nonlinear decides";
    } else {
      if (tr < -eps) kind = "stable degenerate node";
      else if (tr > eps) kind = "unstable degenerate node";
      else kind = "degenerate/center";
    }
  }
  return LinClass{kind, tr, det, disc};
}

static Vec2 rk4_step(double t, const Vec2& z, double h, double a) {
  auto add = [](const Vec2& u, const Vec2& v) { return Vec2{u.x+v.x, u.y+v.y}; };
  auto mul = [](double c, const Vec2& v) { return Vec2{c*v.x, c*v.y}; };

  Vec2 k1 = F(t, z, a);
  Vec2 k2 = F(t + 0.5*h, add(z, mul(0.5*h, k1)), a);
  Vec2 k3 = F(t + 0.5*h, add(z, mul(0.5*h, k2)), a);
  Vec2 k4 = F(t + h, add(z, mul(h, k3)), a);

  Vec2 incr = add(add(mul(1.0, k1), mul(2.0, k2)), add(mul(2.0, k3), mul(1.0, k4)));
  return add(z, mul(h/6.0, incr));
}

int main() {
  const double a = 1.0;

  // Analytical equilibria for a>0:
  // y = x - x^3 and y = -x/a -> x[(a+1)/a - x^2] = 0
  const double xp = std::sqrt((a+1.0)/a);

  std::vector<Vec2> eqs = {
      {0.0, 0.0},
      {+xp, -xp},
      {-xp, +xp}
  };

  std::cout << "Equilibria and local classification (linearization):\n";
  for (const auto& z : eqs) {
    double J[2][2];
    jacobian(z, a, J);
    LinClass cls = classify2x2(J);
    std::cout << "  z* = (" << z.x << ", " << z.y << ")"
              << "  trace=" << cls.tr << " det=" << cls.det
              << " -> " << cls.kind << "\n";
  }

  // Trajectory simulation (RK4) and CSV output
  std::vector<Vec2> initials = {
      {-2.0, -2.0}, {-2.0, 0.0}, {-2.0, 2.0},
      { 0.5, -2.0}, { 0.5, 2.0},
      { 2.0, -2.0}, { 2.0, 0.0}, { 2.0, 2.0}
  };

  const double h = 0.01;
  const double tmax = 12.0;
  const int steps = static_cast<int>(tmax / h);

  std::ofstream out("trajectories.csv");
  out << "traj_id,t,x,y\n";

  for (size_t k = 0; k < initials.size(); ++k) {
    Vec2 z = initials[k];
    double t = 0.0;
    for (int i = 0; i <= steps; ++i) {
      out << k << "," << t << "," << z.x << "," << z.y << "\n";
      z = rk4_step(t, z, h, a);
      t += h;
    }
  }

  std::cout << "\nWrote trajectories.csv (plot it in Python/MATLAB/Excel).\n";
  return 0;
}
