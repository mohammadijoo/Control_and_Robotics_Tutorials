#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Vector2d;

struct TwoRParams {
  double g;
  double l1, l2;
  double lc1, lc2;
  double m1, m2;
  double I1, I2;
};

Matrix2d M_sym(const Vector2d& q, const TwoRParams& p) {
  const double q2 = q(1);
  const double c2 = std::cos(q2);

  const double M11 =
      p.I1 + p.I2 +
      p.m1 * p.lc1 * p.lc1 +
      p.m2 * (p.l1 * p.l1 + p.lc2 * p.lc2 + 2.0 * p.l1 * p.lc2 * c2);

  const double M12 =
      p.I2 + p.m2 * (p.lc2 * p.lc2 + p.l1 * p.lc2 * c2);

  const double M22 = p.I2 + p.m2 * p.lc2 * p.lc2;

  Matrix2d M;
  M(0, 0) = M11;
  M(0, 1) = M12;
  M(1, 0) = M12;
  M(1, 1) = M22;
  return M;
}

Vector2d g_sym(const Vector2d& q, const TwoRParams& p) {
  const double q1 = q(0);
  const double q2 = q(1);

  const double c1    = std::cos(q1);
  const double c12   = std::cos(q1 + q2);

  const double g1 =
      (p.m1 * p.lc1 + p.m2 * p.l1) * p.g * c1 +
      p.m2 * p.lc2 * p.g * c12;

  const double g2 =
      p.m2 * p.lc2 * p.g * c12;

  return Vector2d(g1, g2);
}

int main() {
  TwoRParams p{9.81, 1.0, 1.0, 0.5, 0.5, 2.0, 1.0, 0.1, 0.1};
  Vector2d q;
  q << 0.3, -0.7;

  Matrix2d M = M_sym(q, p);
  Vector2d g = g_sym(q, p);

  std::cout << "M(q):\n" << M << "\n";
  std::cout << "g(q): " << g.transpose() << "\n";
  return 0;
}
      
