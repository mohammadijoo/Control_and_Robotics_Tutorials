// Chapter7_Lesson1.cpp
// Chapter 7 - Lesson 1: EKF Localization Pipeline (mobile framing)
//
// Minimal EKF localization demo (2D) using Eigen.
// Build example (Linux):
//   g++ -O2 -std=c++17 Chapter7_Lesson1.cpp -I/usr/include/eigen3 -o ekf_demo
//
// State: [x, y, theta]^T (world frame), control: [v, omega]^T (body frame)
// Measurement: range-bearing to known landmark.

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

struct Landmark {
  double mx;
  double my;
};

static double wrapAngle(double a) {
  const double twoPi = 2.0 * M_PI;
  a = std::fmod(a + M_PI, twoPi);
  if (a < 0) a += twoPi;
  return a - M_PI;
}

static Eigen::Vector3d motionModel(const Eigen::Vector3d& x,
                                   const Eigen::Vector2d& u,
                                   double dt) {
  const double px = x(0), py = x(1), th = x(2);
  const double v = u(0), w = u(1);

  Eigen::Vector3d nx;
  nx(0) = px + dt * v * std::cos(th);
  nx(1) = py + dt * v * std::sin(th);
  nx(2) = wrapAngle(th + dt * w);
  return nx;
}

static Eigen::Matrix3d jacobianF(const Eigen::Vector3d& x,
                                 const Eigen::Vector2d& u,
                                 double dt) {
  const double th = x(2);
  const double v = u(0);
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0,2) = -dt * v * std::sin(th);
  F(1,2) =  dt * v * std::cos(th);
  return F;
}

static Eigen::Matrix<double,3,2> jacobianL(const Eigen::Vector3d& x, double dt) {
  const double th = x(2);
  Eigen::Matrix<double,3,2> L;
  L.setZero();
  L(0,0) = dt * std::cos(th);
  L(1,0) = dt * std::sin(th);
  L(2,1) = dt;
  return L;
}

static Eigen::Vector2d measRangeBearing(const Eigen::Vector3d& x, const Landmark& lm) {
  const double dx = lm.mx - x(0);
  const double dy = lm.my - x(1);
  const double r = std::sqrt(dx*dx + dy*dy);
  const double b = wrapAngle(std::atan2(dy, dx) - x(2));
  return Eigen::Vector2d(r, b);
}

static Eigen::Matrix<double,2,3> jacobianH(const Eigen::Vector3d& x, const Landmark& lm) {
  const double dx = lm.mx - x(0);
  const double dy = lm.my - x(1);
  double q = dx*dx + dy*dy;
  const double eps = 1e-12;
  if (q < eps) q = eps;
  const double r = std::sqrt(q);

  Eigen::Matrix<double,2,3> H;
  H.setZero();
  // range
  H(0,0) = -dx / r;
  H(0,1) = -dy / r;
  // bearing
  H(1,0) =  dy / q;
  H(1,1) = -dx / q;
  H(1,2) = -1.0;
  return H;
}

int main() {
  // Landmarks
  std::vector<Landmark> lms = {{5.0,0.0},{5.0,5.0},{0.0,5.0},{-3.0,2.0}};

  const double dt = 0.1;
  const int N = 250;

  // Noise parameters
  const double sigma_v_true = 0.05;
  const double sigma_w_true = 0.03;

  Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
  Q(0,0) = 0.08*0.08;
  Q(1,1) = 0.05*0.05;

  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0,0) = 0.15*0.15;
  R(1,1) = std::pow(2.0*M_PI/180.0, 2);

  // Initial truth / estimate
  Eigen::Vector3d x_true(0.0, 0.0, 0.0);
  Eigen::Vector3d x_est(-0.2, 0.1, 0.05);

  Eigen::Matrix3d P = Eigen::Matrix3d::Zero();
  P(0,0) = 0.5*0.5;
  P(1,1) = 0.5*0.5;
  P(2,2) = std::pow(10.0*M_PI/180.0, 2);

  std::mt19937 rng(7);
  std::normal_distribution<double> nv(0.0, sigma_v_true);
  std::normal_distribution<double> nw(0.0, sigma_w_true);
  std::normal_distribution<double> nr(0.0, std::sqrt(R(0,0)));
  std::normal_distribution<double> nb(0.0, std::sqrt(R(1,1)));

  auto nearestLandmarkIndex = [&](const Eigen::Vector3d& x)->int{
    int best = 0;
    double bestd = 1e100;
    for (int i=0;i<(int)lms.size();++i) {
      const double dx = lms[i].mx - x(0);
      const double dy = lms[i].my - x(1);
      const double d = std::sqrt(dx*dx + dy*dy);
      if (d < bestd) { bestd = d; best = i; }
    }
    return best;
  };

  double sumPos2 = 0.0, sumTh2 = 0.0;

  for (int k=0;k<N;++k) {
    const double v_cmd = 0.8 + 0.2 * std::sin(0.04 * k);
    const double w_cmd = 0.25 + 0.05 * std::cos(0.03 * k);

    Eigen::Vector2d u_cmd(v_cmd, w_cmd);
    Eigen::Vector2d u_true(v_cmd + nv(rng), w_cmd + nw(rng));

    // True propagation
    x_true = motionModel(x_true, u_true, dt);

    // EKF predict
    Eigen::Matrix3d F = jacobianF(x_est, u_cmd, dt);
    Eigen::Matrix<double,3,2> L = jacobianL(x_est, dt);
    x_est = motionModel(x_est, u_cmd, dt);
    P = F * P * F.transpose() + L * Q * L.transpose();
    P = 0.5 * (P + P.transpose());

    // Measurement (nearest landmark)
    const int idx = nearestLandmarkIndex(x_true);
    const Landmark lm = lms[idx];

    Eigen::Vector2d z_true = measRangeBearing(x_true, lm);
    Eigen::Vector2d z_meas(z_true(0) + nr(rng), wrapAngle(z_true(1) + nb(rng)));

    // EKF update
    Eigen::Matrix<double,2,3> H = jacobianH(x_est, lm);
    Eigen::Vector2d zhat = measRangeBearing(x_est, lm);
    Eigen::Vector2d y = z_meas - zhat;
    y(1) = wrapAngle(y(1));

    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix<double,3,2> K = P * H.transpose() * S.inverse();

    x_est = x_est + K * y;
    x_est(2) = wrapAngle(x_est(2));

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    P = (I - K*H) * P * (I - K*H).transpose() + K * R * K.transpose();
    P = 0.5 * (P + P.transpose());

    // Errors
    const double ex = x_est(0) - x_true(0);
    const double ey = x_est(1) - x_true(1);
    const double eth = wrapAngle(x_est(2) - x_true(2));
    sumPos2 += ex*ex + ey*ey;
    sumTh2 += eth*eth;
  }

  const double rmsPos = std::sqrt(sumPos2 / (double)N);
  const double rmsTh  = std::sqrt(sumTh2 / (double)N);

  std::cout << "RMS position error [m]: " << rmsPos << "\n";
  std::cout << "RMS heading error [rad]: " << rmsTh << "\n";
  return 0;
}
