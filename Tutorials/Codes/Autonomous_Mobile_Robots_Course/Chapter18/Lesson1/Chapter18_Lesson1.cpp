// Chapter18_Lesson1.cpp
// Autonomous Mobile Robots (Control Engineering) — Chapter 18, Lesson 1
// GPS/RTK Integration in Navigation (didactic EKF fusion example)
//
// This C++17 example mirrors the Python EKF:
//   state x = [px, py, yaw, v, bg]^T
//   inputs: wheel speed v_meas, gyro yaw-rate omega_meas
//   meas  : GNSS/RTK position (px, py) in ENU
//
// Requires Eigen3. Build (example):
//   g++ -O2 -std=c++17 Chapter18_Lesson1.cpp -I /usr/include/eigen3 -o ekf_gnss

#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>

#include <Eigen/Dense>

static double wrap_to_pi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  a -= M_PI;
  // map -pi to +pi
  if (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

struct EkfConfig {
  double q_xy  = 0.05;
  double q_yaw = 0.01;
  double q_v   = 0.5;
  double q_bg  = 1e-4;

  double r_v = 0.2 * 0.2;

  double r_gps_fix = 1.5 * 1.5;
  double r_gps_rtk = 0.02 * 0.02;

  // chi2(2) 0.99
  double gate_chi2_dof2 = 9.2103;
};

class EkfGpsFusion {
public:
  explicit EkfGpsFusion(EkfConfig cfg = EkfConfig()) : cfg_(cfg) {
    x_.setZero();
    P_.setZero();
    P_(0,0) = 10.0;
    P_(1,1) = 10.0;
    P_(2,2) = std::pow(20.0 * M_PI / 180.0, 2);
    P_(3,3) = 2.0 * 2.0;
    P_(4,4) = std::pow(5.0 * M_PI / 180.0, 2);
  }

  void set_state(double px, double py, double yaw, double v, double bg) {
    x_ << px, py, yaw, v, bg;
  }

  void predict(double v_meas, double omega_meas, double dt) {
    if (dt <= 0.0) return;
    const double px = x_(0), py = x_(1), yaw = x_(2), v = x_(3), bg = x_(4);
    const double omega = omega_meas - bg;

    const double v_pred = v; // random walk
    const double yaw_pred = wrap_to_pi(yaw + omega * dt);
    const double px_pred = px + v_pred * std::cos(yaw_pred) * dt;
    const double py_pred = py + v_pred * std::sin(yaw_pred) * dt;

    Eigen::Matrix<double,5,5> F = Eigen::Matrix<double,5,5>::Identity();
    F(0,2) = -v_pred * std::sin(yaw_pred) * dt;
    F(0,3) =  std::cos(yaw_pred) * dt;
    F(1,2) =  v_pred * std::cos(yaw_pred) * dt;
    F(1,3) =  std::sin(yaw_pred) * dt;
    F(2,4) = -dt;

    Eigen::Matrix<double,5,5> Q = Eigen::Matrix<double,5,5>::Zero();
    Q(0,0) = cfg_.q_xy  * dt;
    Q(1,1) = cfg_.q_xy  * dt;
    Q(2,2) = cfg_.q_yaw * dt;
    Q(3,3) = cfg_.q_v   * dt;
    Q(4,4) = cfg_.q_bg  * dt;

    x_ << px_pred, py_pred, yaw_pred, v_pred, bg;
    P_ = F * P_ * F.transpose() + Q;

    update_speed(v_meas);
  }

  void update_speed(double v_meas) {
    Eigen::Matrix<double,1,5> H; H.setZero();
    H(0,3) = 1.0;
    const double z = v_meas;
    const double h = x_(3);
    const double y = z - h;

    Eigen::Matrix<double,1,1> S = H * P_ * H.transpose();
    S(0,0) += cfg_.r_v;

    Eigen::Matrix<double,5,1> K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::Matrix<double,5,5>::Identity() - K * H) * P_;
  }

  bool update_gnss(double px_meas, double py_meas, double r_pos) {
    Eigen::Matrix<double,2,5> H; H.setZero();
    H(0,0) = 1.0;
    H(1,1) = 1.0;

    Eigen::Matrix<double,2,1> z;
    z << px_meas, py_meas;

    Eigen::Matrix<double,2,1> h;
    h << x_(0), x_(1);

    Eigen::Matrix<double,2,2> R = Eigen::Matrix<double,2,2>::Zero();
    R(0,0) = r_pos; R(1,1) = r_pos;

    Eigen::Matrix<double,2,1> innov = z - h;
    Eigen::Matrix<double,2,2> S = H * P_ * H.transpose() + R;

    const double d2 = (innov.transpose() * S.inverse() * innov)(0,0);
    if (d2 > cfg_.gate_chi2_dof2) return false;

    Eigen::Matrix<double,5,2> K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * innov;
    x_(2) = wrap_to_pi(x_(2));
    P_ = (Eigen::Matrix<double,5,5>::Identity() - K * H) * P_;
    return true;
  }

  const Eigen::Matrix<double,5,1>& x() const { return x_; }

private:
  EkfConfig cfg_;
  Eigen::Matrix<double,5,1> x_;
  Eigen::Matrix<double,5,5> P_;
};

static double rtk_quality_to_rpos(int fix_q, const EkfConfig& cfg) {
  if (fix_q == 4) return cfg.r_gps_rtk;     // RTK fixed
  if (fix_q == 5) return 0.2 * 0.2;         // RTK float
  if (fix_q == 2) return 0.8 * 0.8;         // DGPS
  return cfg.r_gps_fix;                     // autonomous GNSS
}

int main() {
  EkfConfig cfg;
  EkfGpsFusion ekf(cfg);
  ekf.set_state(0.0, 0.0, 0.0, 1.0, 0.0);

  const double dt = 0.05;
  const double T = 60.0;
  const int n = static_cast<int>(T / dt);

  // Truth
  const double v_true = 1.2;
  const double omega_true = 0.07;
  const double bg_true = 0.02;

  std::mt19937 rng(42);
  std::normal_distribution<double> n_v(0.0, 0.05);
  std::normal_distribution<double> n_omega(0.0, 0.01);

  double px = 0.0, py = 0.0, yaw = 0.0;

  std::cout << "t,true_x,true_y,true_yaw,est_x,est_y,est_yaw,gnss_ok\n";

  for (int k = 0; k < n; ++k) {
    const double t = k * dt;

    // Truth update
    yaw = wrap_to_pi(yaw + omega_true * dt);
    px += v_true * std::cos(yaw) * dt;
    py += v_true * std::sin(yaw) * dt;

    // Measurements
    const double v_meas = v_true + n_v(rng);
    const double omega_meas = omega_true + bg_true + n_omega(rng);

    ekf.predict(v_meas, omega_meas, dt);

    bool gnss_ok = false;
    if (k % static_cast<int>(0.2 / dt) == 0) {
      const int fix_q = (k % 200 != 0) ? 4 : 5;
      const double r_pos = rtk_quality_to_rpos(fix_q, cfg);

      std::normal_distribution<double> n_g(0.0, std::sqrt(r_pos));
      const double gnss_px = px + n_g(rng);
      const double gnss_py = py + n_g(rng);

      gnss_ok = ekf.update_gnss(gnss_px, gnss_py, r_pos);
    }

    const auto& xhat = ekf.x();
    std::cout << t << "," << px << "," << py << "," << yaw << ","
              << xhat(0) << "," << xhat(1) << "," << xhat(2) << ","
              << (gnss_ok ? 1 : 0) << "\n";
  }
  return 0;
}
