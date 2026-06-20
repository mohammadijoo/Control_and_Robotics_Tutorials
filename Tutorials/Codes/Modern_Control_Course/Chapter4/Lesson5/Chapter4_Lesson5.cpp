#include <iostream>
#include <complex>
#include <vector>
#include <Eigen/Dense>

int main() {
  using std::complex;
  using Eigen::Matrix2d;
  using Eigen::Vector2d;
  using Eigen::RowVector2d;
  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  // Parameters
  const double m = 1.0, b = 0.4, k = 4.0;

  Matrix2d A;
  A << 0.0, 1.0,
      -k/m, -b/m;
  Vector2d B;
  B << 0.0, 1.0/m;
  RowVector2d C;
  C << 1.0, 0.0;
  const double D = 0.0;

  // Frequency response comparison
  std::vector<double> w = {0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0};
  for (double wi : w) {
    complex<double> s(0.0, wi);

    // Compute G(s) = C (sI - A)^{-1} B + D
    Eigen::Matrix<complex<double>, 2, 2> M;
    M << s - A(0,0), -A(0,1),
         -A(1,0),     s - A(1,1);

    Eigen::Matrix<complex<double>, 2, 2> Minv = M.inverse();
    Eigen::Matrix<complex<double>, 2, 1> Bc;
    Bc << B(0), B(1);
    Eigen::Matrix<complex<double>, 1, 2> Cc;
    Cc << C(0), C(1);

    complex<double> G_ss = (Cc * Minv * Bc)(0,0) + D;

    // Transfer function polynomial route: 1 / (m s^2 + b s + k)
    complex<double> G_tf = 1.0 / (m*s*s + b*s + k);

    std::cout << "w=" << wi
              << "  G_ss=" << G_ss
              << "  G_tf=" << G_tf
              << "  diff=" << std::abs(G_ss - G_tf)
              << std::endl;
  }

  // Simple time simulation by RK4 on xdot = A x + B u, step input u=1
  auto f = [&](const Vector2d& x, double u) {
    return A*x + B*u;
  };

  const double dt = 0.001;
  const int N = 20000; // 20 seconds
  Vector2d x; x << 0.0, 0.0; // x(0)=0
  for (int i = 0; i < N; ++i) {
    double u = 1.0;
    Vector2d k1 = f(x, u);
    Vector2d k2 = f(x + 0.5*dt*k1, u);
    Vector2d k3 = f(x + 0.5*dt*k2, u);
    Vector2d k4 = f(x + dt*k3, u);
    x = x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);

    if (i % 2000 == 0) {
      double y = (C*x)(0) + D*u;
      std::cout << "t=" << (i*dt) << "  y=" << y << std::endl;
    }
  }

  return 0;
}
