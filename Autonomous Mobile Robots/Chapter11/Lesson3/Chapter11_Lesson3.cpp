// Chapter11_Lesson3.cpp
/*
FastSLAM 1.0 (Rao–Blackwellized PF-SLAM) — compact C++ reference implementation.

Educational notes:
- 2D pose: (x,y,theta)
- Known landmark IDs in observations
- Each particle maintains EKF (mean+cov) per landmark
- Uses Eigen for linear algebra (recommended for robotics)

Build (example):
  g++ -O2 -std=c++17 Chapter11_Lesson3.cpp -I /usr/include/eigen3 -o fastslam_demo

This file focuses on the algorithmic core (no ROS, no visualization).
*/

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

static double wrapAngle(double a) {
  while (a <= -M_PI) a += 2.0 * M_PI;
  while (a > M_PI) a -= 2.0 * M_PI;
  return a;
}

struct LandmarkEKF {
  bool seen = false;
  Eigen::Vector2d mu = Eigen::Vector2d::Zero();
  Eigen::Matrix2d Sigma = 1e6 * Eigen::Matrix2d::Identity();
};

struct Particle {
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  double w = 1.0;
  std::vector<LandmarkEKF> lms;

  explicit Particle(int M) : lms(M) {}
};

struct Obs {
  int id;
  Eigen::Vector2d z; // [range, bearing]
};

static Eigen::Vector3d motionModel(const Eigen::Vector3d& x, const Eigen::Vector2d& u, double dt) {
  double px = x(0), py = x(1), th = x(2);
  double v = u(0), w = u(1);

  Eigen::Vector3d x2;
  if (std::abs(w) < 1e-9) {
    x2(0) = px + v * dt * std::cos(th);
    x2(1) = py + v * dt * std::sin(th);
    x2(2) = th;
  } else {
    x2(0) = px + (v / w) * (std::sin(th + w * dt) - std::sin(th));
    x2(1) = py - (v / w) * (std::cos(th + w * dt) - std::cos(th));
    x2(2) = th + w * dt;
  }
  x2(2) = wrapAngle(x2(2));
  return x2;
}

static Eigen::Vector2d hLandmark(const Eigen::Vector3d& x, const Eigen::Vector2d& m) {
  double px = x(0), py = x(1), th = x(2);
  double dx = m(0) - px;
  double dy = m(1) - py;
  double q = dx*dx + dy*dy;
  double r = std::sqrt(std::max(q, 1e-12));
  double b = wrapAngle(std::atan2(dy, dx) - th);
  return Eigen::Vector2d(r, b);
}

static Eigen::Matrix2d H_landmark(const Eigen::Vector3d& x, const Eigen::Vector2d& m) {
  double px = x(0), py = x(1), th = x(2);
  (void)th;
  double dx = m(0) - px;
  double dy = m(1) - py;
  double q = std::max(dx*dx + dy*dy, 1e-12);
  double r = std::sqrt(std::max(q, 1e-12));
  Eigen::Matrix2d H;
  H << dx/r, dy/r,
      -dy/q, dx/q;
  return H;
}

static Eigen::Vector2d invMeasurement(const Eigen::Vector3d& x, const Eigen::Vector2d& z) {
  double px = x(0), py = x(1), th = x(2);
  double r = z(0), b = z(1);
  double ang = th + b;
  return Eigen::Vector2d(px + r * std::cos(ang), py + r * std::sin(ang));
}

static double gaussLikelihood(const Eigen::Vector2d& v, const Eigen::Matrix2d& S) {
  Eigen::Matrix2d Ssym = 0.5 * (S + S.transpose());
  double detS = std::max(Ssym.determinant(), 1e-18);
  Eigen::Matrix2d invS = Ssym.inverse();
  double e = v.transpose() * invS * v;
  double norm = 1.0 / (2.0 * M_PI * std::sqrt(detS));
  return norm * std::exp(-0.5 * e);
}

static void normalizeWeights(std::vector<Particle>& ps) {
  double sumw = 0.0;
  for (auto& p : ps) sumw += p.w;
  if (sumw < 1e-30) {
    double w0 = 1.0 / ps.size();
    for (auto& p : ps) p.w = w0;
    return;
  }
  for (auto& p : ps) p.w /= sumw;
}

static double effectiveSampleSize(const std::vector<Particle>& ps) {
  double s2 = 0.0;
  for (const auto& p : ps) s2 += p.w * p.w;
  return (s2 > 1e-18) ? 1.0 / s2 : 0.0;
}

static std::vector<Particle> systematicResample(const std::vector<Particle>& ps, std::mt19937& gen) {
  int N = (int)ps.size();
  std::vector<double> cdf(N, 0.0);
  cdf[0] = ps[0].w;
  for (int i = 1; i < N; ++i) cdf[i] = cdf[i-1] + ps[i].w;

  std::uniform_real_distribution<double> uni(0.0, 1.0 / N);
  double u0 = uni(gen);

  std::vector<Particle> out;
  out.reserve(N);

  int j = 0;
  for (int i = 0; i < N; ++i) {
    double u = u0 + (double)i / (double)N;
    while (j < N-1 && u > cdf[j]) j++;
    Particle pnew = ps[j]; // copy includes landmarks
    pnew.w = 1.0 / N;
    out.push_back(std::move(pnew));
  }
  return out;
}

static void fastslamStep(std::vector<Particle>& ps,
                         const Eigen::Vector2d& u,
                         const std::vector<Obs>& obs,
                         double dt,
                         const Eigen::Matrix2d& R_u,
                         const Eigen::Matrix2d& Q_z,
                         std::mt19937& gen) {
  std::normal_distribution<double> n01(0.0, 1.0);

  // 1) motion sampling by perturbing controls
  for (auto& p : ps) {
    Eigen::Vector2d noise;
    noise(0) = n01(gen);
    noise(1) = n01(gen);
    Eigen::Matrix2d L = R_u.llt().matrixL();
    Eigen::Vector2d u_noisy = u + L * noise;
    p.x = motionModel(p.x, u_noisy, dt);
  }

  // 2) landmark EKF updates + weights
  for (auto& p : ps) {
    for (const auto& o : obs) {
      auto& lm = p.lms[o.id];
      if (!lm.seen) {
        lm.mu = invMeasurement(p.x, o.z);
        Eigen::Matrix2d H = H_landmark(p.x, lm.mu);
        Eigen::Matrix2d J = H.inverse();
        lm.Sigma = J * Q_z * J.transpose();
        lm.seen = true;
        continue;
      }

      Eigen::Vector2d zhat = hLandmark(p.x, lm.mu);
      Eigen::Vector2d v;
      v(0) = o.z(0) - zhat(0);
      v(1) = wrapAngle(o.z(1) - zhat(1));

      Eigen::Matrix2d H = H_landmark(p.x, lm.mu);
      Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + Q_z;
      Eigen::Matrix2d K = lm.Sigma * H.transpose() * S.inverse();
      lm.mu = lm.mu + K * v;
      lm.Sigma = (Eigen::Matrix2d::Identity() - K * H) * lm.Sigma;

      p.w *= gaussLikelihood(v, S);
    }
  }

  normalizeWeights(ps);

  double ess = effectiveSampleSize(ps);
  if (ess < 0.5 * ps.size()) {
    ps = systematicResample(ps, gen);
  }
}

int main() {
  // Simple sanity check run: no world simulation here; just compile + run.
  // You can plug in your own simulator and fill obs each step.
  const int M = 5;       // landmarks
  const int N = 100;     // particles
  const int T = 10;
  const double dt = 0.1;

  Eigen::Matrix2d R_u = Eigen::Matrix2d::Zero();
  R_u(0,0) = 0.05 * 0.05;
  R_u(1,1) = std::pow(2.0 * M_PI / 180.0, 2);

  Eigen::Matrix2d Q_z = Eigen::Matrix2d::Zero();
  Q_z(0,0) = 0.15 * 0.15;
  Q_z(1,1) = std::pow(3.0 * M_PI / 180.0, 2);

  std::vector<Particle> ps;
  ps.reserve(N);
  for (int i = 0; i < N; ++i) {
    Particle p(M);
    p.x = Eigen::Vector3d(0.0, 0.0, 0.0);
    p.w = 1.0 / N;
    ps.push_back(p);
  }

  std::mt19937 gen(42);

  for (int t = 0; t < T; ++t) {
    Eigen::Vector2d u(0.8, 0.1);

    // Fake observations: none (for demo). Replace with real obs from simulator/sensor.
    std::vector<Obs> obs;

    fastslamStep(ps, u, obs, dt, R_u, Q_z, gen);

    // Weighted mean pose
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    double c = 0.0, s = 0.0;
    for (const auto& p : ps) {
      mean(0) += p.w * p.x(0);
      mean(1) += p.w * p.x(1);
      c += p.w * std::cos(p.x(2));
      s += p.w * std::sin(p.x(2));
    }
    mean(2) = std::atan2(s, c);

    std::cout << "t=" << t << " mean pose: " << mean.transpose() << "\n";
  }

  return 0;
}
