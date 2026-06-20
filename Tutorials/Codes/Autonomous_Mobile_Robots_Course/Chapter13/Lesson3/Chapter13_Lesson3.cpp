// Chapter13_Lesson3.cpp
// Visual–Inertial Fusion Pipelines (AMR Focus)
//
// Minimal planar EKF fusion (IMU propagation + visual pose updates).
// Dependencies: Eigen (for matrices).
//
// Build example:
//   g++ -O2 -std=c++17 Chapter13_Lesson3.cpp -I /usr/include/eigen3 -o vio_ekf
//
// Pedagogical baseline, not production VIO.

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

static inline double wrapAngle(double th) {
  th = std::fmod(th + M_PI, 2.0 * M_PI);
  if (th < 0) th += 2.0 * M_PI;
  return th - M_PI;
}

static inline Eigen::Matrix2d rot2(double th) {
  double c = std::cos(th), s = std::sin(th);
  Eigen::Matrix2d R;
  R << c, -s,
       s,  c;
  return R;
}

struct TruthSample {
  double t;
  double px, py, vx, vy, th;
  double axm, aym, wzm; // IMU measurements
};

struct VOMeas {
  double t;
  double px, py, th;
};

static void simulate(double T, double dt, double vo_dt,
                     std::vector<TruthSample>& samples,
                     std::vector<VOMeas>& vo) {
  std::mt19937 rng(7);
  std::normal_distribution<double> n01(0.0, 1.0);

  int N = static_cast<int>(T / dt) + 1;
  samples.resize(N);

  Eigen::Vector2d g(0.0, -9.81);

  std::vector<double> px(N, 0.0), py(N, 0.0), vx(N, 0.0), vy(N, 0.0), th(N, 0.0);

  auto speed = [&](double t) { return 1.0 + 0.2 * std::sin(0.3 * t); };
  auto yawrate = [&](double t) { return 0.25 * std::sin(0.2 * t); };

  for (int k = 1; k < N; ++k) {
    double tk = (k - 1) * dt;
    th[k] = wrapAngle(th[k - 1] + yawrate(tk) * dt);
    vx[k] = speed(tk) * std::cos(th[k]);
    vy[k] = speed(tk) * std::sin(th[k]);
    px[k] = px[k - 1] + vx[k - 1] * dt;
    py[k] = py[k - 1] + vy[k - 1] * dt;
  }

  std::vector<double> axw(N, 0.0), ayw(N, 0.0);
  for (int k = 1; k < N - 1; ++k) {
    axw[k] = (vx[k + 1] - vx[k - 1]) / (2.0 * dt);
    ayw[k] = (vy[k + 1] - vy[k - 1]) / (2.0 * dt);
  }
  axw[0] = axw[1]; ayw[0] = ayw[1];
  axw[N - 1] = axw[N - 2]; ayw[N - 1] = ayw[N - 2];

  Eigen::Vector2d b_a(0.08, -0.05);
  double b_g = 0.01;
  double sigma_a = 0.12;
  double sigma_g = 0.015;

  for (int k = 0; k < N; ++k) {
    double tk = k * dt;
    Eigen::Matrix2d R = rot2(th[k]);
    Eigen::Vector2d a_w(axw[k], ayw[k]);
    Eigen::Vector2d a_b = R.transpose() * (a_w - g);
    double axm = a_b.x() + b_a.x() + sigma_a * n01(rng);
    double aym = a_b.y() + b_a.y() + sigma_a * n01(rng);
    double wzm = yawrate(tk) + b_g + sigma_g * n01(rng);

    samples[k] = TruthSample{tk, px[k], py[k], vx[k], vy[k], th[k], axm, aym, wzm};
  }

  int vo_step = static_cast<int>(vo_dt / dt);
  double sigma_p = 0.05;
  double sigma_th = 0.02;
  for (int k = 0; k < N; k += vo_step) {
    VOMeas m;
    m.t = samples[k].t;
    m.px = samples[k].px + sigma_p * n01(rng);
    m.py = samples[k].py + sigma_p * n01(rng);
    m.th = wrapAngle(samples[k].th + sigma_th * n01(rng));
    vo.push_back(m);
  }
}

static void imuPropagate(Eigen::VectorXd& x, Eigen::MatrixXd& P,
                         const Eigen::Vector3d& u, const Eigen::MatrixXd& Qc,
                         double dt) {
  double px = x(0), py = x(1), vx = x(2), vy = x(3), th = x(4);
  double bax = x(5), bay = x(6), bg = x(7);
  double axm = u(0), aym = u(1), wzm = u(2);

  Eigen::Vector2d g(0.0, -9.81);

  Eigen::Vector2d a_b(axm - bax, aym - bay);
  double w = wzm - bg;

  double th_new = wrapAngle(th + w * dt);
  Eigen::Matrix2d R = rot2(th);
  Eigen::Vector2d a_w = R * a_b + g;

  double vx_new = vx + a_w.x() * dt;
  double vy_new = vy + a_w.y() * dt;
  double px_new = px + vx * dt + 0.5 * a_w.x() * dt * dt;
  double py_new = py + vy * dt + 0.5 * a_w.y() * dt * dt;

  x << px_new, py_new, vx_new, vy_new, th_new, bax, bay, bg;

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(8, 8);
  F(0, 2) = dt;
  F(1, 3) = dt;

  Eigen::Matrix2d dR_dth;
  dR_dth << -std::sin(th), -std::cos(th),
             std::cos(th), -std::sin(th);
  Eigen::Vector2d da_w_dth = dR_dth * a_b;

  F(2, 4) = da_w_dth.x() * dt;
  F(3, 4) = da_w_dth.y() * dt;

  F(2, 5) = -R(0, 0) * dt;
  F(2, 6) = -R(0, 1) * dt;
  F(3, 5) = -R(1, 0) * dt;
  F(3, 6) = -R(1, 1) * dt;

  F(0, 4) = da_w_dth.x() * 0.5 * dt * dt;
  F(1, 4) = da_w_dth.y() * 0.5 * dt * dt;
  F(0, 5) = -R(0, 0) * 0.5 * dt * dt;
  F(0, 6) = -R(0, 1) * 0.5 * dt * dt;
  F(1, 5) = -R(1, 0) * 0.5 * dt * dt;
  F(1, 6) = -R(1, 1) * 0.5 * dt * dt;

  F(4, 7) = -dt;

  Eigen::MatrixXd Qd = Qc * dt;
  P = F * P * F.transpose() + Qd;
}

static void ekfUpdatePose(Eigen::VectorXd& x, Eigen::MatrixXd& P,
                          const Eigen::Vector3d& z, const Eigen::Matrix3d& Rz) {
  Eigen::Matrix<double, 3, 8> H = Eigen::Matrix<double, 3, 8>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  H(2, 4) = 1.0;

  Eigen::Vector3d zhat(x(0), x(1), x(4));
  Eigen::Vector3d y = z - zhat;
  y(2) = wrapAngle(y(2));

  Eigen::Matrix3d S = H * P * H.transpose() + Rz;
  Eigen::Matrix<double, 8, 3> K = P * H.transpose() * S.inverse();

  x = x + K * y;
  x(4) = wrapAngle(x(4));

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(8, 8);
  P = (I - K * H) * P * (I - K * H).transpose() + K * Rz * K.transpose();
}

int main() {
  double dt = 0.01;
  double T = 20.0;
  double vo_dt = 0.1;

  std::vector<TruthSample> samples;
  std::vector<VOMeas> vo;
  simulate(T, dt, vo_dt, samples, vo);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(8);
  x(0) = vo[0].px;
  x(1) = vo[0].py;
  x(4) = vo[0].th;

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(8, 8);
  P.diagonal() << 0.25, 0.25, 0.04, 0.04, 0.09, 0.04, 0.04, 0.01;

  Eigen::MatrixXd Qc = Eigen::MatrixXd::Zero(8, 8);
  Qc(2, 2) = 0.4 * 0.4;
  Qc(3, 3) = 0.4 * 0.4;
  Qc(4, 4) = 0.15 * 0.15;
  Qc(5, 5) = 0.01 * 0.01;
  Qc(6, 6) = 0.01 * 0.01;
  Qc(7, 7) = 0.002 * 0.002;

  Eigen::Matrix3d Rz = Eigen::Matrix3d::Zero();
  Rz(0, 0) = 0.05 * 0.05;
  Rz(1, 1) = 0.05 * 0.05;
  Rz(2, 2) = 0.02 * 0.02;

  int vo_ptr = 0;
  for (size_t k = 0; k < samples.size(); ++k) {
    Eigen::Vector3d u(samples[k].axm, samples[k].aym, samples[k].wzm);
    imuPropagate(x, P, u, Qc, dt);

    if (vo_ptr < static_cast<int>(vo.size()) &&
        std::abs(samples[k].t - vo[vo_ptr].t) < 0.5 * dt) {
      Eigen::Vector3d z(vo[vo_ptr].px, vo[vo_ptr].py, vo[vo_ptr].th);
      ekfUpdatePose(x, P, z, Rz);
      vo_ptr++;
    }
  }

  const TruthSample& gt = samples.back();
  Eigen::Vector3d err(x(0) - gt.px, x(1) - gt.py, wrapAngle(x(4) - gt.th));
  std::cout << "Final error [m, m, rad]: " << err.transpose() << std::endl;
  return 0;
}
