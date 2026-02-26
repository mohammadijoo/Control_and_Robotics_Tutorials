// Chapter11_Lesson1.cpp
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 11: SLAM I — Filter-Based SLAM
Lesson 1: The SLAM Problem Formulation

A compact EKF-SLAM core (known data association) for a 2D unicycle robot with
range-bearing landmark observations.

Build note:
  - Requires Eigen (header-only). Example compile:
    g++ -O2 -std=c++17 Chapter11_Lesson1.cpp -I /path/to/eigen -o slam_demo
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

static double wrapAngle(double a) {
  const double twoPi = 2.0 * M_PI;
  a = std::fmod(a + M_PI, twoPi);
  if (a < 0) a += twoPi;
  return a - M_PI;
}

static Eigen::Vector3d motionModel(const Eigen::Vector3d& x, const Eigen::Vector3d& u) {
  const double px = x(0), py = x(1), th = x(2);
  const double v = u(0), w = u(1), dt = u(2);

  Eigen::Vector3d xp = x;
  if (std::abs(w) < 1e-12) {
    xp(0) = px + v * dt * std::cos(th);
    xp(1) = py + v * dt * std::sin(th);
    xp(2) = th;
  } else {
    const double th2 = th + w * dt;
    xp(0) = px + (v / w) * (std::sin(th2) - std::sin(th));
    xp(1) = py - (v / w) * (std::cos(th2) - std::cos(th));
    xp(2) = th2;
  }
  xp(2) = wrapAngle(xp(2));
  return xp;
}

static Eigen::Matrix3d jacMotionWrtState(const Eigen::Vector3d& x, const Eigen::Vector3d& u) {
  const double th = x(2);
  const double v = u(0), w = u(1), dt = u(2);

  Eigen::Matrix3d Fx = Eigen::Matrix3d::Identity();
  if (std::abs(w) < 1e-12) {
    Fx(0,2) = -v * dt * std::sin(th);
    Fx(1,2) =  v * dt * std::cos(th);
  } else {
    const double th2 = th + w * dt;
    Fx(0,2) = (v / w) * (std::cos(th2) - std::cos(th));
    Fx(1,2) = (v / w) * (std::sin(th2) - std::sin(th));
  }
  return Fx;
}

static Eigen::Vector2d measModel(const Eigen::Vector3d& x, const Eigen::Vector2d& m) {
  const double px = x(0), py = x(1), th = x(2);
  const double dx = m(0) - px;
  const double dy = m(1) - py;
  const double r = std::sqrt(dx*dx + dy*dy);
  const double b = wrapAngle(std::atan2(dy, dx) - th);
  return Eigen::Vector2d(r, b);
}

static void jacMeasWrtPoseLandmark(
  const Eigen::Vector3d& x, const Eigen::Vector2d& m,
  Eigen::Matrix<double,2,3>& Hx,
  Eigen::Matrix2d& Hm
) {
  const double px = x(0), py = x(1);
  const double dx = m(0) - px;
  const double dy = m(1) - py;
  double q = dx*dx + dy*dy;
  if (q < 1e-12) q = 1e-12;
  const double r = std::sqrt(q);

  Hx << -dx/r, -dy/r, 0.0,
         dy/q, -dx/q, -1.0;

  Hm <<  dx/r,  dy/r,
        -dy/q,  dx/q;
}

int main() {
  // Three landmarks in the world (unknown to the filter)
  std::vector<Eigen::Vector2d> landmarks = {
    {4.0,  2.0},
    {8.0, -1.0},
    {2.0, -3.0}
  };
  const int N = static_cast<int>(landmarks.size());
  const int n = 3 + 2*N;

  // Augmented state y = [x, m1, m2, m3]
  Eigen::VectorXd y(n);
  y.setZero();
  y.segment<3>(0) = Eigen::Vector3d(0.0, 0.0, 0.0);
  y.segment<6>(3) = Eigen::VectorXd::Map((double[]){3.0,1.0, 7.0,0.0, 1.0,-2.0}, 6);

  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n,n) * 1e-3;
  for (int i=0;i<N;i++) {
    P.block<2,2>(3+2*i, 3+2*i) = Eigen::Matrix2d::Identity() * 4.0;
  }

  Eigen::Matrix3d Qpose = Eigen::Vector3d(0.02*0.02, 0.02*0.02, std::pow(M_PI/180.0,2)).asDiagonal();
  Eigen::Matrix2d Rmeas = Eigen::Vector2d(0.10*0.10, std::pow(2.0*M_PI/180.0,2)).asDiagonal();

  // Single step demo control
  Eigen::Vector3d u(1.0, 0.10, 1.0);

  // --- Predict ---
  Eigen::Vector3d x = y.segment<3>(0);
  Eigen::Matrix3d Fx = jacMotionWrtState(x, u);
  Eigen::Vector3d xPred = motionModel(x, u);

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n,n);
  F.block<3,3>(0,0) = Fx;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n,n);
  Q.block<3,3>(0,0) = Qpose;

  y.segment<3>(0) = xPred;
  P = F * P * F.transpose() + Q;

  // --- Update using landmark 0 (known association) ---
  const int lmId = 0;
  const int idx = 3 + 2*lmId;
  Eigen::Vector2d mi = y.segment<2>(idx);

  // Suppose we measured the true landmark with small noise
  Eigen::Vector2d z = measModel(xPred, landmarks[lmId]);
  z(0) += 0.05;     // range noise
  z(1) = wrapAngle(z(1) + 1.0 * M_PI/180.0); // bearing noise

  Eigen::Vector2d zHat = measModel(y.segment<3>(0), mi);
  Eigen::Vector2d innov(z(0) - zHat(0), wrapAngle(z(1) - zHat(1)));

  Eigen::Matrix<double,2,3> Hx;
  Eigen::Matrix2d Hm;
  jacMeasWrtPoseLandmark(y.segment<3>(0), mi, Hx, Hm);

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2,n);
  H.block<2,3>(0,0) = Hx;
  H.block<2,2>(0,idx) = Hm;

  Eigen::Matrix2d S = H * P * H.transpose() + Rmeas;
  Eigen::MatrixXd K = P * H.transpose() * S.inverse();

  y = y + K * innov;
  y(2) = wrapAngle(y(2));

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n,n);
  P = (I - K*H) * P * (I - K*H).transpose() + K * Rmeas * K.transpose();

  std::cout << "Updated pose mean: " << y.segment<3>(0).transpose() << "\n";
  std::cout << "Updated landmark0 mean: " << y.segment<2>(idx).transpose() << "\n";
  return 0;
}
