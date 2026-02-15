
#include <Eigen/Dense>
#include <vector>

using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

struct ILQROptions {
  int N;
  int maxIter;
  double dt;
};

Vector2d dynamics(const Vector2d& x, double u, double dt) {
  double m = 1.0, l = 1.0, g = 9.81;
  double theta = x(0);
  double dtheta = x(1);

  double ddtheta = (-g / l) * std::sin(theta) + u / (m * l * l);
  Vector2d xnext;
  xnext(0) = theta + dt * dtheta;
  xnext(1) = dtheta + dt * ddtheta;
  return xnext;
}

void linearizeDynamics(const Vector2d& x, double /*u*/, double dt,
                       Matrix2d& fx, Eigen::Vector2d& fu) {
  double m = 1.0, l = 1.0, g = 9.81;
  double theta = x(0);

  double ddtheta_dtheta = (-g / l) * std::cos(theta);
  double ddtheta_du = 1.0 / (m * l * l);

  fx.setIdentity();
  fx(0, 1) = dt;
  fx(1, 0) = dt * ddtheta_dtheta;
  fx(1, 1) = 1.0;

  fu.setZero();
  fu(1) = dt * ddtheta_du;
}

void ilqr(const Vector2d& x0,
          const std::vector<Vector2d>& xRef,
          const ILQROptions& opt,
          std::vector<Vector2d>& X,
          std::vector<double>& U) {
  int N = opt.N;
  double dt = opt.dt;

  X.assign(N + 1, Vector2d::Zero());
  U.assign(N, 0.0);
  X[0] = x0;

  // Initial rollout
  for (int k = 0; k < N; ++k) {
    X[k + 1] = dynamics(X[k], U[k], dt);
  }

  Matrix2d Q, P_N;
  Q.setZero();
  Q(0, 0) = 10.0;
  Q(1, 1) = 1.0;
  P_N.setZero();
  P_N(0, 0) = 50.0;
  P_N(1, 1) = 5.0;
  double R = 0.1;

  for (int it = 0; it < opt.maxIter; ++it) {
    // Backward pass
    Eigen::Vector2d Vx = P_N * (X[N] - xRef[N]);
    Matrix2d Vxx = P_N;

    std::vector<Eigen::Matrix<double, 1, 2> > K(N);
    std::vector<double> k_ff(N);

    double reg = 1e-6;

    for (int k = N - 1; k >= 0; --k) {
      const Vector2d& xk = X[k];
      double uk = U[k];
      const Vector2d& xref = xRef[k];

      Matrix2d fx;
      Eigen::Vector2d fu;
      linearizeDynamics(xk, uk, dt, fx, fu);

      Vector2d dx = xk - xref;
      Vector2d lx = Q * dx;
      double lu = R * uk;
      Matrix2d lxx = Q;
      double luu = R;

      Vector2d Qx = lx + fx.transpose() * Vx;
      double Qu = lu + fu.transpose() * Vx;
      Matrix2d Qxx = lxx + fx.transpose() * Vxx * fx;
      double Quu = luu + (fu.transpose() * Vxx * fu)(0) + reg;
      Eigen::RowVector2d Qux = (fu.transpose() * Vxx * fx);

      double Quu_inv = 1.0 / Quu;
      double k_local = -Quu_inv * Qu;
      Eigen::RowVector2d K_local = -Quu_inv * Qux;

      k_ff[k] = k_local;
      K[k] = K_local;

      Vx = Qx + K_local.transpose() * Quu * k_local
          + K_local.transpose() * Qu + Qux.transpose() * k_local;
      Vxx = Qxx + K_local.transpose() * Quu * K_local
           + K_local.transpose() * Qux + Qux.transpose() * K_local;
      Vxx = 0.5 * (Vxx + Vxx.transpose());
    }

    // Forward pass not fully shown (similar to Python version)
    // ...
  }
}
