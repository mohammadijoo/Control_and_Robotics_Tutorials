// Chapter2_Lesson1.cpp
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 2, Lesson 1: Rolling Constraints and Instantaneous Motion

Build and run (example):
  g++ -O2 -std=c++17 Chapter2_Lesson1.cpp -I /path/to/eigen -o lesson1
  ./lesson1

This code:
- Encodes lateral no-slip constraints A * xi = 0 for standard wheels
- Computes wheel spin rates from the rolling constraint
- Computes the ICR (instantaneous center of rotation) in the body frame
*/

#include <iostream>
#include <vector>
#include <cmath>

#include &lt;Eigen/Dense&gt;

struct Wheel {
  double lx;     // body-frame x position
  double ly;     // body-frame y position
  double alpha;  // rolling direction angle (rad)
  double R;      // radius
};

static Eigen::Vector2d t_dir(double a) {
  return Eigen::Vector2d(std::cos(a), std::sin(a));
}

static Eigen::Vector2d n_dir(double a) {
  return Eigen::Vector2d(-std::sin(a), std::cos(a));
}

// v(r) = v + omega k x r, with k x [x,y] = [-y, x]
static Eigen::Vector2d v_point(double vx, double vy, double omega, const Eigen::Vector2d& r) {
  return Eigen::Vector2d(vx, vy) + omega * Eigen::Vector2d(-r.y(), r.x());
}

static Eigen::RowVector3d lateral_constraint_row(const Wheel& w) {
  Eigen::Vector2d r(w.lx, w.ly);
  Eigen::Vector2d n = n_dir(w.alpha);
  // n^T [vx, vy] + omega * n^T [-ry, rx] = 0
  double c = (-r.y() * n.x() + r.x() * n.y());
  return Eigen::RowVector3d(n.x(), n.y(), c);
}

static Eigen::MatrixXd buildA(const std::vector&lt;Wheel&gt;& wheels) {
  Eigen::MatrixXd A(wheels.size(), 3);
  for (size_t i = 0; i &lt; wheels.size(); ++i) {
    A.row(static_cast&lt;int&gt;(i)) = lateral_constraint_row(wheels[i]);
  }
  return A;
}

static std::vector&lt;double&gt; wheelSpinRates(const std::vector&lt;Wheel&gt;& wheels,
                                         double vx, double vy, double omega) {
  std::vector&lt;double&gt; rates;
  rates.reserve(wheels.size());
  for (const auto& w : wheels) {
    Eigen::Vector2d r(w.lx, w.ly);
    Eigen::Vector2d t = t_dir(w.alpha);
    Eigen::Vector2d vpt = v_point(vx, vy, omega, r);
    rates.push_back(t.dot(vpt) / w.R);
  }
  return rates;
}

// ICR (body): p = [-vy/omega, vx/omega], omega != 0
static bool icrBody(double vx, double vy, double omega, Eigen::Vector2d& out_p) {
  const double eps = 1e-12;
  if (std::abs(omega) &lt; eps) return false;
  out_p = Eigen::Vector2d(-vy / omega, vx / omega);
  return true;
}

// residual = (p - r) x n = (dx*ny - dy*nx)
static double axleLineResidual(const Wheel& w, const Eigen::Vector2d& p) {
  Eigen::Vector2d r(w.lx, w.ly);
  Eigen::Vector2d n = n_dir(w.alpha);
  Eigen::Vector2d d = p - r;
  return d.x() * n.y() - d.y() * n.x();
}

int main() {
  std::cout &lt;&lt; "=== Demo: two parallel wheels (constraints + ICR) ===\n";
  double b = 0.6;
  std::vector&lt;Wheel&gt; wheels = {
    {0.0, +b/2, 0.0, 0.1},
    {0.0, -b/2, 0.0, 0.1}
  };

  double vx = 0.5, vy = 0.0, omega = 0.8;
  Eigen::Vector3d xi(vx, vy, omega);

  Eigen::MatrixXd A = buildA(wheels);
  std::cout &lt;&lt; "A =\n" &lt;&lt; A &lt;&lt; "\n";
  std::cout &lt;&lt; "A*xi = " &lt;&lt; (A * xi).transpose() &lt;&lt; "\n";

  Eigen::Vector2d p;
  if (icrBody(vx, vy, omega, p)) {
    std::cout &lt;&lt; "ICR (body) = [" &lt;&lt; p.x() &lt;&lt; ", " &lt;&lt; p.y() &lt;&lt; "]\n";
    for (size_t i = 0; i &lt; wheels.size(); ++i) {
      std::cout &lt;&lt; "Axle-line residual wheel " &lt;&lt; (i+1) &lt;&lt; ": "
                &lt;&lt; axleLineResidual(wheels[i], p) &lt;&lt; "\n";
    }
  } else {
    std::cout &lt;&lt; "omega ~ 0, ICR at infinity (pure translation)\n";
  }

  auto rates = wheelSpinRates(wheels, vx, vy, omega);
  std::cout &lt;&lt; "Wheel spin rates [rad/s]: ";
  for (double r : rates) std::cout &lt;&lt; r &lt;&lt; " ";
  std::cout &lt;&lt; "\n";

  return 0;
}
