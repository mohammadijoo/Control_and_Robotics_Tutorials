/*
Chapter 8 - Particle-Filter Localization
Lesson 1: Monte Carlo Localization (MCL) Intuition

Minimal from-scratch MCL in a known 2D landmark map (ranges).
Dependencies: Eigen (header-only)

Compile (example):
  g++ -O2 -std=c++17 Chapter8_Lesson1.cpp -I/usr/include/eigen3 -o mcl_demo
*/

#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <numeric>
#include <algorithm>

#include <Eigen/Dense>

struct Particle {
  Eigen::Vector3d x;  // [x, y, theta]
  double w;
};

static double wrap_angle(double a) {
  const double pi = 3.14159265358979323846;
  a = std::fmod(a + pi, 2.0 * pi);
  if (a < 0.0) a += 2.0 * pi;
  return a - pi;
}

static Eigen::Vector3d motion_sample(
    const Eigen::Vector3d& x, const Eigen::Vector2d& u, double dt,
    double sigma_v, double sigma_omega,
    std::mt19937& gen)
{
  std::normal_distribution<double> nv(0.0, sigma_v);
  std::normal_distribution<double> nw(0.0, sigma_omega);

  const double v = u(0) + nv(gen);
  const double w = u(1) + nw(gen);

  Eigen::Vector3d xn = x;
  const double th = x(2);

  if (std::abs(w) < 1e-9) {
    xn(0) += v * dt * std::cos(th);
    xn(1) += v * dt * std::sin(th);
  } else {
    xn(0) += (v / w) * (std::sin(th + w * dt) - std::sin(th));
    xn(1) += (v / w) * (-std::cos(th + w * dt) + std::cos(th));
    xn(2) = wrap_angle(th + w * dt);
  }
  return xn;
}

static Eigen::VectorXd expected_ranges(
    const Eigen::Vector3d& x,
    const std::vector<Eigen::Vector2d>& landmarks)
{
  Eigen::VectorXd r(landmarks.size());
  for (size_t i = 0; i < landmarks.size(); ++i) {
    const double dx = landmarks[i](0) - x(0);
    const double dy = landmarks[i](1) - x(1);
    r(i) = std::sqrt(dx*dx + dy*dy);
  }
  return r;
}

static double gaussian_logpdf(const Eigen::VectorXd& e, double sigma) {
  const double pi = 3.14159265358979323846;
  const double n = static_cast<double>(e.size());
  return -0.5 * (e.array() / sigma).square().sum() - n * std::log(sigma * std::sqrt(2.0 * pi));
}

static std::vector<Particle> systematic_resample(
    const std::vector<Particle>& P,
    std::mt19937& gen)
{
  const int N = static_cast<int>(P.size());
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  std::vector<double> cdf(N);
  cdf[0] = P[0].w;
  for (int i = 1; i < N; ++i) cdf[i] = cdf[i-1] + P[i].w;
  cdf.back() = 1.0;

  const double u0 = uni(gen) / static_cast<double>(N);
  std::vector<Particle> out;
  out.reserve(N);

  int i = 0;
  for (int m = 0; m < N; ++m) {
    const double u = u0 + static_cast<double>(m) / static_cast<double>(N);
    while (u > cdf[i] && i < N-1) ++i;
    Particle q = P[i];
    q.w = 1.0 / static_cast<double>(N);
    out.push_back(q);
  }
  return out;
}

static Eigen::Vector3d mean_pose(const std::vector<Particle>& P) {
  Eigen::Vector3d mu = Eigen::Vector3d::Zero();
  double s = 0.0;
  double c = 0.0;
  for (const auto& p : P) {
    mu(0) += p.x(0);
    mu(1) += p.x(1);
    s += std::sin(p.x(2));
    c += std::cos(p.x(2));
  }
  mu(0) /= static_cast<double>(P.size());
  mu(1) /= static_cast<double>(P.size());
  mu(2) = std::atan2(s / static_cast<double>(P.size()), c / static_cast<double>(P.size()));
  return mu;
}

int main() {
  std::mt19937 gen(0);
  std::normal_distribution<double> n0(0.0, 1.0);
  std::uniform_real_distribution<double> unif(-3.14159265358979323846, 3.14159265358979323846);

  // Landmarks: square corners
  std::vector<Eigen::Vector2d> lm = {
    {-5.0, -5.0}, {5.0, -5.0}, {5.0, 5.0}, {-5.0, 5.0}
  };

  Eigen::Vector3d x_true(-3.0, -2.0, 0.3);

  const int N = 800;
  const double sigma_v = 0.10, sigma_omega = 0.05, sigma_r = 0.35;
  const double dt = 0.1;

  // Initialize particles
  std::vector<Particle> P(N);
  for (int i = 0; i < N; ++i) {
    P[i].x(0) = 3.0 * n0(gen);
    P[i].x(1) = 3.0 * n0(gen);
    P[i].x(2) = unif(gen);
    P[i].w = 1.0 / static_cast<double>(N);
  }

  std::normal_distribution<double> nr(0.0, sigma_r);

  // Run
  const int T = 120;
  for (int t = 0; t < T; ++t) {
    const Eigen::Vector2d u(0.6, 0.25);  // v, omega (commanded)

    // True pose (noise-free for demo)
    x_true = motion_sample(x_true, u, dt, 0.0, 0.0, gen);

    // Range measurement (noisy)
    Eigen::VectorXd z = expected_ranges(x_true, lm);
    for (int i = 0; i < z.size(); ++i) z(i) += nr(gen);

    // Predict
    for (auto& p : P) p.x = motion_sample(p.x, u, dt, sigma_v, sigma_omega, gen);

    // Weight update
    std::vector<double> logw(N);
    double maxlogw = -1e300;
    for (int i = 0; i < N; ++i) {
      const Eigen::VectorXd zhat = expected_ranges(P[i].x, lm);
      const Eigen::VectorXd e = z - zhat;
      logw[i] = gaussian_logpdf(e, sigma_r);
      if (logw[i] > maxlogw) maxlogw = logw[i];
    }

    double sumw = 0.0;
    for (int i = 0; i < N; ++i) {
      P[i].w = std::exp(logw[i] - maxlogw);
      sumw += P[i].w;
    }
    for (auto& p : P) p.w /= sumw;

    // Resample
    P = systematic_resample(P, gen);

    // Report estimate
    const Eigen::Vector3d mu = mean_pose(P);
    std::cout << "t=" << t
              << " true=(" << x_true(0) << "," << x_true(1) << "," << x_true(2) << ")"
              << " est=("  << mu(0)     << "," << mu(1)     << "," << mu(2)     << ")"
              << "\n";
  }

  std::cout << "Done.\n";
  return 0;
}
