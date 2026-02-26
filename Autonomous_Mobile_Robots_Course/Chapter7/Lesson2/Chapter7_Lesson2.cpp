/*
Chapter7_Lesson2.cpp
UKF localization for planar (x, y, theta) robot with range-bearing landmarks.

Dependencies:
  - Eigen3 (header-only linear algebra library)

Build (example):
  g++ -O2 -std=c++17 Chapter7_Lesson2.cpp -I /usr/include/eigen3 -o ukf_demo

This file implements a minimal additive-noise UKF from scratch and runs a small simulation.
*/

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

static double wrap_angle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

static double mean_angle(const Eigen::VectorXd& angles, const Eigen::VectorXd& w) {
  double s = 0.0, c = 0.0;
  for (int i = 0; i < angles.size(); ++i) {
    s += w(i) * std::sin(angles(i));
    c += w(i) * std::cos(angles(i));
  }
  return wrap_angle(std::atan2(s, c));
}

struct UKFParams {
  double alpha{0.35};
  double beta{2.0};
  double kappa{0.0};
};

class UKF {
public:
  static constexpr int n = 3; // [x, y, theta]

  explicit UKF(const UKFParams& p) : params(p) {
    lambda = params.alpha * params.alpha * (n + params.kappa) - n;

    Wm = Eigen::VectorXd::Constant(2 * n + 1, 1.0 / (2.0 * (n + lambda)));
    Wc = Wm;
    Wm(0) = lambda / (n + lambda);
    Wc(0) = Wm(0) + (1.0 - params.alpha * params.alpha + params.beta);

    x.setZero();
    P.setIdentity();
    Q.setIdentity(); Q *= 1e-3;
  }

  void setState(const Eigen::Vector3d& x0, const Eigen::Matrix3d& P0) {
    x = x0;
    x(2) = wrap_angle(x(2));
    P = P0;
  }

  void setProcessNoise(const Eigen::Matrix3d& Q_) { Q = Q_; }

  template<typename MotionFcn>
  void predict(MotionFcn f, const Eigen::Vector2d& u) {
    Eigen::Matrix<double, 2 * n + 1, n> X = sigmaPoints(x, P);
    Eigen::Matrix<double, 2 * n + 1, n> Xp;

    for (int i = 0; i < X.rows(); ++i) {
      Eigen::Vector3d xi = X.row(i).transpose();
      Eigen::Vector3d yi = f(xi, u);
      yi(2) = wrap_angle(yi(2));
      Xp.row(i) = yi.transpose();
    }

    Eigen::Vector3d x_pred = stateMean(Xp);
    Eigen::Matrix3d P_pred = Eigen::Matrix3d::Zero();
    for (int i = 0; i < Xp.rows(); ++i) {
      Eigen::Vector3d dx = stateResidual(Xp.row(i).transpose(), x_pred);
      P_pred += Wc(i) * (dx * dx.transpose());
    }
    P_pred += Q;

    x = x_pred;
    P = P_pred;
  }

  template<typename MeasFcn>
  void updateSequential(const Eigen::Vector2d& z, MeasFcn h, const Eigen::Matrix2d& R) {
    Eigen::Matrix<double, 2 * n + 1, n> X = sigmaPoints(x, P);

    // measurement sigma points
    Eigen::Matrix<double, 2 * n + 1, 2> Z;
    for (int i = 0; i < X.rows(); ++i) {
      Eigen::Vector3d xi = X.row(i).transpose();
      Eigen::Vector2d zi = h(xi);
      zi(1) = wrap_angle(zi(1));
      Z.row(i) = zi.transpose();
    }

    // mean measurement (bearing is circular)
    Eigen::Vector2d z_pred;
    z_pred(0) = (Wm.array() * Z.col(0).array()).sum();
    z_pred(1) = mean_angle(Z.col(1), Wm);

    // covariances
    Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
    Eigen::Matrix<double, n, 2> Pxz = Eigen::Matrix<double, n, 2>::Zero();
    for (int i = 0; i < Z.rows(); ++i) {
      Eigen::Vector2d dz = Z.row(i).transpose() - z_pred;
      dz(1) = wrap_angle(dz(1));
      Eigen::Vector3d dx = stateResidual(X.row(i).transpose(), x);
      S += Wc(i) * (dz * dz.transpose());
      Pxz += Wc(i) * (dx * dz.transpose());
    }
    S += R;

    Eigen::Matrix<double, n, 2> K = Pxz * S.inverse();
    Eigen::Vector2d innov = z - z_pred;
    innov(1) = wrap_angle(innov(1));

    x = x + K * innov;
    x(2) = wrap_angle(x(2));
    P = P - K * S * K.transpose();
  }

  Eigen::Vector3d x;
  Eigen::Matrix3d P;

private:
  UKFParams params;
  double lambda{0.0};
  Eigen::VectorXd Wm, Wc;
  Eigen::Matrix3d Q;

  Eigen::Matrix<double, 2 * n + 1, n> sigmaPoints(const Eigen::Vector3d& x, const Eigen::Matrix3d& P) const {
    Eigen::Matrix3d S = ((n + lambda) * P).llt().matrixL();
    Eigen::Matrix<double, 2 * n + 1, n> X;
    X.row(0) = x.transpose();
    for (int i = 0; i < n; ++i) {
      X.row(i + 1) = (x + S.col(i)).transpose();
      X.row(i + 1 + n) = (x - S.col(i)).transpose();
    }
    // wrap theta sigma points
    for (int i = 0; i < X.rows(); ++i) X(i, 2) = wrap_angle(X(i, 2));
    return X;
  }

  Eigen::Vector3d stateMean(const Eigen::Matrix<double, 2 * n + 1, n>& X) const {
    Eigen::Vector3d m;
    m(0) = (Wm.array() * X.col(0).array()).sum();
    m(1) = (Wm.array() * X.col(1).array()).sum();
    m(2) = mean_angle(X.col(2), Wm);
    return m;
  }

  Eigen::Vector3d stateResidual(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
    Eigen::Vector3d y = a - b;
    y(2) = wrap_angle(y(2));
    return y;
  }
};

static Eigen::Vector3d motionModel(const Eigen::Vector3d& x, const Eigen::Vector2d& u, double dt) {
  double px = x(0), py = x(1), th = x(2);
  double v = u(0), w = u(1);
  Eigen::Vector3d y;
  y(0) = px + dt * v * std::cos(th);
  y(1) = py + dt * v * std::sin(th);
  y(2) = wrap_angle(th + dt * w);
  return y;
}

static Eigen::Vector2d measModel(const Eigen::Vector3d& x, const Eigen::Vector2d& lm) {
  double px = x(0), py = x(1), th = x(2);
  double dx = lm(0) - px;
  double dy = lm(1) - py;
  double r = std::sqrt(dx * dx + dy * dy);
  double b = wrap_angle(std::atan2(dy, dx) - th);
  return Eigen::Vector2d(r, b);
}

int main() {
  std::mt19937 rng(7);
  std::normal_distribution<double> n01(0.0, 1.0);

  const double dt = 0.1;
  const int T = 250;

  std::vector<Eigen::Vector2d> landmarks = {
    {5.0, 0.0},
    {0.0, 6.0},
    {6.0, 6.0},
    {8.0, -2.0}
  };

  Eigen::Vector3d x_true(0.0, 0.0, 0.2);

  UKF ukf(UKFParams{0.35, 2.0, 0.0});
  Eigen::Vector3d x0(0.5, -0.5, -0.3);
  Eigen::Matrix3d P0 = Eigen::Matrix3d::Zero();
  P0(0,0) = 0.8 * 0.8;
  P0(1,1) = 0.8 * 0.8;
  P0(2,2) = std::pow(20.0 * M_PI / 180.0, 2.0);
  ukf.setState(x0, P0);

  double sigma_xy = 0.02;
  double sigma_th = 1.0 * M_PI / 180.0;
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(0,0) = sigma_xy * sigma_xy;
  Q(1,1) = sigma_xy * sigma_xy;
  Q(2,2) = sigma_th * sigma_th;
  ukf.setProcessNoise(Q);

  double sigma_r = 0.15;
  double sigma_b = 2.0 * M_PI / 180.0;
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0,0) = sigma_r * sigma_r;
  R(1,1) = sigma_b * sigma_b;

  for (int k = 0; k < T; ++k) {
    double v = 1.0 + 0.2 * std::sin(0.07 * k);
    double w = 0.35 * std::sin(0.03 * k);
    Eigen::Vector2d u(v, w);

    // true propagation with process noise
    x_true = motionModel(x_true, u, dt);
    x_true(0) += sigma_xy * n01(rng);
    x_true(1) += sigma_xy * n01(rng);
    x_true(2) = wrap_angle(x_true(2) + sigma_th * n01(rng));

    // UKF predict
    ukf.predict([&](const Eigen::Vector3d& x, const Eigen::Vector2d& uu) {
      return motionModel(x, uu, dt);
    }, u);

    // sequential landmark updates
    for (const auto& lm : landmarks) {
      Eigen::Vector2d z = measModel(x_true, lm);
      z(0) += sigma_r * n01(rng);
      z(1) = wrap_angle(z(1) + sigma_b * n01(rng));

      ukf.updateSequential(z, [&](const Eigen::Vector3d& x) {
        return measModel(x, lm);
      }, R);
    }
  }

  std::cout << "Final true state: " << x_true.transpose() << "\n";
  std::cout << "Final UKF  state: " << ukf.x.transpose() << "\n";
  std::cout << "Final UKF covariance diag: " << ukf.P.diagonal().transpose() << "\n";
  return 0;
}
