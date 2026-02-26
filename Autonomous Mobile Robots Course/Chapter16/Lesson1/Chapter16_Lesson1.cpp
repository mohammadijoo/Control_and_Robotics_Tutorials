/*
Chapter16_Lesson1.cpp
Sensing Dynamic Obstacles — Minimal Kalman Filter + Gating (C++ / Eigen)

This file provides:
- 2D constant-velocity (CV) Kalman predict/update
- Chi-square gating using Mahalanobis distance
- A small demo with synthetic measurements

Build (example):
  g++ -O2 -std=c++17 Chapter16_Lesson1.cpp -I /usr/include/eigen3 -o demo

Notes:
- In robotics stacks, use Eigen + ROS2 (rclcpp) and consume detections from perception.
- For LiDAR clustering, typical libraries include PCL; for vision use OpenCV.
*/

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>

struct KF {
  Eigen::Vector4d x;     // [px, py, vx, vy]
  Eigen::Matrix4d P;
  Eigen::Matrix4d F;
  Eigen::Matrix4d Q;
  Eigen::Matrix<double,2,4> H;
  Eigen::Matrix2d R;

  KF(double dt, double sigma_a, double sigma_z) {
    F.setIdentity();
    F(0,2) = dt; F(1,3) = dt;

    const double q = sigma_a * sigma_a;
    Q.setZero();
    Q(0,0) = q * std::pow(dt,4)/4.0; Q(0,2) = q * std::pow(dt,3)/2.0;
    Q(1,1) = q * std::pow(dt,4)/4.0; Q(1,3) = q * std::pow(dt,3)/2.0;
    Q(2,0) = q * std::pow(dt,3)/2.0; Q(2,2) = q * std::pow(dt,2);
    Q(3,1) = q * std::pow(dt,3)/2.0; Q(3,3) = q * std::pow(dt,2);

    H.setZero();
    H(0,0) = 1.0; H(1,1) = 1.0;

    R.setZero();
    R(0,0) = sigma_z * sigma_z;
    R(1,1) = sigma_z * sigma_z;

    x.setZero();
    P.setIdentity();
    P(0,0)=1; P(1,1)=1; P(2,2)=2; P(3,3)=2;
  }

  void predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  // Returns Mahalanobis distance squared d^2
  double update(const Eigen::Vector2d& z) {
    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix<double,4,2> K = P * H.transpose() * S.inverse();

    x = x + K * y;

    // Joseph form for numerical stability
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

    const double d2 = y.transpose() * S.inverse() * y;
    return d2;
  }

  Eigen::Vector2d zhat() const { return H * x; }

  double maha2(const Eigen::Vector2d& z) const {
    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    return y.transpose() * S.inverse() * y;
  }
};

// For dof=2, common chi-square gates (approx).
double chi2_gate_2dof(double pg) {
  if (std::abs(pg - 0.99) < 1e-12) return 9.210;
  if (std::abs(pg - 0.95) < 1e-12) return 5.991;
  if (std::abs(pg - 0.90) < 1e-12) return 4.605;
  return 9.210;
}

int main() {
  const double dt = 0.1;
  const double sigma_a = 0.7;
  const double sigma_z = 0.35;
  const double gate = chi2_gate_2dof(0.99);

  KF kf(dt, sigma_a, sigma_z);

  // Truth: start at (0,0), velocity (1.0, 0.6)
  Eigen::Vector4d x_true; x_true << 0, 0, 1.0, 0.6;

  std::mt19937 rng(0);
  std::normal_distribution<double> n01(0.0, 1.0);

  for (int k=0; k<60; ++k) {
    // propagate truth
    x_true(0) += dt * x_true(2);
    x_true(1) += dt * x_true(3);

    // noisy position measurement
    Eigen::Vector2d z;
    z(0) = x_true(0) + sigma_z * n01(rng);
    z(1) = x_true(1) + sigma_z * n01(rng);

    kf.predict();
    const double d2 = kf.maha2(z);

    if (d2 <= gate) {
      kf.update(z);
      std::cout << "k=" << k
                << "  z=[" << z(0) << "," << z(1) << "]"
                << "  d2=" << d2
                << "  xhat=[" << kf.x(0) << "," << kf.x(1) << "," << kf.x(2) << "," << kf.x(3) << "]\n";
    } else {
      std::cout << "k=" << k << "  measurement rejected by gate, d2=" << d2 << "\n";
    }
  }

  return 0;
}
