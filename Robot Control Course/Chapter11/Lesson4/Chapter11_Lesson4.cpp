
#include <iostream>
#include <deque>
#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::RowVector3d;

struct Sample {
  double t;
  double y;   // encoder reading
};

int main() {
  double J = 0.05;
  double d = 0.01;
  double Kp = 50.0;
  double Kd = 2.0 * std::sqrt(Kp * J);

  double dt = 0.001;
  double h  = 0.02;
  int delaySteps = static_cast<int>(h / dt);

  Matrix3d A;
  A << 0.0, 1.0, 0.0,
         0.0, -d / J, 0.0,
         0.0, 0.0,  0.0;
  Eigen::Vector3d B;
  B << 0.0, 1.0 / J, 0.0;
  RowVector3d C;
  C << 1.0, 0.0, 1.0;

  Matrix3d Q = Matrix3d::Zero();
  Q(0,0) = 1e-6; Q(1,1) = 1e-4; Q(2,2) = 1e-8;
  double R = 1e-4;

  Vector3d x_hat(0.0, 0.0, 0.0);
  Matrix3d P = Matrix3d::Identity() * 1e-2;

  auto q_ref = [](double t) {
    return 0.5 * std::sin(2.0 * M_PI * 0.5 * t);
  };

  std::deque<Sample> meas_buffer;
  double t = 0.0;
  double x_true_q = 0.0;
  double x_true_qd = 0.0;
  double b_true = 0.05;

  for (int k = 0; k < 10000; ++k) {
    double qr = q_ref(t);
    double e  = qr - x_hat(0);
    double ed = 0.0 - x_hat(1);
    double u  = Kp * e + Kd * ed;

    // True dynamics (Euler)
    double qdd = (-d * x_true_qd + u) / J;
    x_true_q  += dt * x_true_qd;
    x_true_qd += dt * qdd;

    // Measurement at time t (will be used delayed)
    double noise = std::sqrt(R) * ((double)std::rand() / RAND_MAX - 0.5);
    double y = x_true_q + b_true + noise;
    meas_buffer.push_back({t, y});
    if (meas_buffer.size() > static_cast<size_t>(delaySteps + 2)) {
      meas_buffer.pop_front();
    }

    // EKF predict
    Matrix3d Ad = Matrix3d::Identity() + dt * A;
    Vector3d Bd = dt * B;
    x_hat = Ad * x_hat + Bd * u;
    P = Ad * P * Ad.transpose() + Q;

    // EKF update with delayed measurement
    if ((int)meas_buffer.size() > delaySteps) {
      Sample y_delayed = meas_buffer[meas_buffer.size() - delaySteps - 1];
      double innov = y_delayed.y - C * x_hat;
      double S = (C * P * C.transpose())(0,0) + R;
      Vector3d K_gain = P * C.transpose() / S;
      x_hat += K_gain * innov;
      P = (Matrix3d::Identity() - K_gain * C) * P;
    }

    t += dt;
  }

  std::cout << "Estimated bias: " << x_hat(2) << std::endl;
  return 0;
}
