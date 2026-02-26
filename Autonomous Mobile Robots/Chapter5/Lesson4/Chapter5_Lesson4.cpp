// Chapter5_Lesson4.cpp
// Autonomous Mobile Robots — Chapter 5 Lesson 4: Practical Odometry Filtering
//
// Demonstrates:
//  - Hampel outlier suppression (median + MAD)
//  - First-order low-pass filter for velocities
//  - Complementary fusion for heading: wheel heading (low-freq) + gyro integration (high-freq)
//  - Planar pose integration
//
// Notes on robotics integration:
//  - In ROS2, subscribe to sensor_msgs::msg::Imu (yaw rate) and encoder-derived twist,
//    publish nav_msgs::msg::Odometry with filtered pose/twist.
//  - For message synchronization and time stamps, consider message_filters.
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter5_Lesson4.cpp -o odo_filter
//
// Output:
//   Writes CSV "chapter5_lesson4_out.csv" with columns: t,x,y,theta,v_f,w_f

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

static double wrapToPi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

struct FirstOrderLowPass {
  double alpha;
  double y;
  bool initialized;
  explicit FirstOrderLowPass(double alpha_) : alpha(alpha_), y(0.0), initialized(false) {}

  static FirstOrderLowPass fromCutoff(double fc_hz, double dt) {
    const double tau = 1.0 / (2.0 * M_PI * fc_hz);
    const double alpha = std::exp(-dt / tau);
    return FirstOrderLowPass(alpha);
  }

  double step(double x) {
    if (!initialized) {
      y = x;
      initialized = true;
      return y;
    }
    y = alpha * y + (1.0 - alpha) * x;
    return y;
  }
};

static double median(std::vector<double> w) {
  std::nth_element(w.begin(), w.begin() + w.size() / 2, w.end());
  return w[w.size() / 2];
}

static double mad(const std::vector<double>& w, double med) {
  std::vector<double> d(w.size());
  for (size_t i = 0; i < w.size(); ++i) d[i] = std::abs(w[i] - med);
  return median(d);
}

static std::vector<double> hampelFilter(const std::vector<double>& x, int window, double nSigmas) {
  if (window % 2 == 0) throw std::runtime_error("window must be odd");
  const int k = window / 2;
  std::vector<double> y = x;

  for (int i = k; i < static_cast<int>(x.size()) - k; ++i) {
    std::vector<double> w;
    w.reserve(window);
    for (int j = i - k; j <= i + k; ++j) w.push_back(x[j]);

    const double med = median(w);
    const double MAD = mad(w, med) + 1e-12;
    const double sigma = 1.4826 * MAD;
    if (std::abs(x[i] - med) > nSigmas * sigma) y[i] = med;
  }
  return y;
}

struct ComplementaryHeadingFilter {
  double alpha;
  double thetaHat;
  bool initialized;

  explicit ComplementaryHeadingFilter(double alpha_) : alpha(alpha_), thetaHat(0.0), initialized(false) {}

  static ComplementaryHeadingFilter fromCutoff(double fc_hz, double dt) {
    const double tau = 1.0 / (2.0 * M_PI * fc_hz);
    const double alpha = std::exp(-dt / tau);
    return ComplementaryHeadingFilter(alpha);
  }

  double step(double omegaGyro, double thetaWheel, double dt, double alphaEff) {
    if (!initialized) {
      thetaHat = thetaWheel;
      initialized = true;
      return thetaHat;
    }
    const double pred = wrapToPi(thetaHat + dt * omegaGyro);
    thetaHat = wrapToPi(alphaEff * pred + (1.0 - alphaEff) * thetaWheel);
    return thetaHat;
  }
};

struct Config {
  double dt = 0.01;
  double vFcHz = 5.0;
  double wFcHz = 8.0;
  double thetaFcHz = 0.7;
  double vMax = 2.0;
  double wMax = 3.0;
  double slipGate = 1.2;
};

int main() {
  Config cfg;
  const double T = 20.0;
  const int N = static_cast<int>(T / cfg.dt);

  std::vector<double> t(N), vTrue(N), wTrue(N), thetaTrue(N, 0.0);
  for (int k = 0; k < N; ++k) {
    t[k] = k * cfg.dt;
    if (t[k] >= 1.0 && t[k] < 6.0) {
      vTrue[k] = 1.1;
      wTrue[k] = 0.0;
    } else if (t[k] >= 6.0 && t[k] < 12.0) {
      vTrue[k] = 0.8;
      wTrue[k] = 0.35;
    } else if (t[k] >= 12.0 && t[k] < 16.0) {
      vTrue[k] = 0.0;
      wTrue[k] = -0.6;
    } else if (t[k] >= 16.0 && t[k] < 20.0) {
      vTrue[k] = 1.0;
      wTrue[k] = 0.15;
    } else {
      vTrue[k] = 0.0;
      wTrue[k] = 0.0;
    }
  }
  for (int k = 1; k < N; ++k) thetaTrue[k] = wrapToPi(thetaTrue[k - 1] + cfg.dt * wTrue[k]);

  std::mt19937 rng(2);
  std::normal_distribution<double> n01(0.0, 1.0);

  // Wheel measurements (noise + quantization + spikes)
  std::vector<double> vW(N), wW(N);
  const double qv = 0.02, qw = 0.02;
  for (int k = 0; k < N; ++k) {
    vW[k] = vTrue[k] + 0.05 * n01(rng);
    wW[k] = wTrue[k] + 0.08 * n01(rng);
    vW[k] = std::round(vW[k] / qv) * qv;
    wW[k] = std::round(wW[k] / qw) * qw;
  }
  // inject spikes
  for (int s = 0; s < 12; ++s) {
    const int idx = rng() % N;
    wW[idx] += 2.0 * n01(rng);
    vW[idx] += 0.8 * n01(rng);
  }

  // Wheel heading from integrating wheel yaw rate
  std::vector<double> thetaW(N, 0.0);
  for (int k = 1; k < N; ++k) thetaW[k] = wrapToPi(thetaW[k - 1] + cfg.dt * wW[k]);

  // Gyro yaw rate: bias + noise + random walk bias drift
  std::vector<double> wG(N), bias(N);
  bias[0] = 0.03;
  for (int k = 1; k < N; ++k) bias[k] = bias[k - 1] + 0.0005 * n01(rng);
  for (int k = 0; k < N; ++k) wG[k] = wTrue[k] + bias[k] + 0.05 * n01(rng);

  // Robust prefilter
  const std::vector<double> vWh = hampelFilter(vW, 11, 3.0);
  const std::vector<double> wWh = hampelFilter(wW, 11, 3.0);

  FirstOrderLowPass vLP = FirstOrderLowPass::fromCutoff(cfg.vFcHz, cfg.dt);
  FirstOrderLowPass wLP = FirstOrderLowPass::fromCutoff(cfg.wFcHz, cfg.dt);
  ComplementaryHeadingFilter thetaCF = ComplementaryHeadingFilter::fromCutoff(cfg.thetaFcHz, cfg.dt);

  double x = 0.0, y = 0.0, theta = 0.0;

  std::ofstream out("chapter5_lesson4_out.csv");
  out << "t,x,y,theta,v_f,w_f\n";

  for (int k = 0; k < N; ++k) {
    double v = std::clamp(vWh[k], -cfg.vMax, cfg.vMax);
    double ww = std::clamp(wWh[k], -cfg.wMax, cfg.wMax);
    double wg = std::clamp(wG[k], -cfg.wMax, cfg.wMax);

    const double resid = std::abs(wg - ww);
    const double alphaEff = (resid > cfg.slipGate) ? std::min(0.995, thetaCF.alpha + 0.15) : thetaCF.alpha;

    const double vF = vLP.step(v);
    const double wF = wLP.step(ww);

    theta = thetaCF.step(wg, thetaW[k], cfg.dt, alphaEff);

    x += cfg.dt * vF * std::cos(theta);
    y += cfg.dt * vF * std::sin(theta);

    out << t[k] << "," << x << "," << y << "," << theta << "," << vF << "," << wF << "\n";
  }

  out.close();
  std::cout << "Wrote chapter5_lesson4_out.csv\n";
  return 0;
}
