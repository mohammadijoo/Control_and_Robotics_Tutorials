// Chapter9_Lesson3.cpp
// Feature-Based Mapping (Landmarks) with per-landmark EKF (pose assumed known).
// Dependencies: Eigen (header-only) for small matrix operations.
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter9_Lesson3.cpp -I /path/to/eigen -o lesson3
// Run:
//   ./lesson3
//
// This program prints the final estimated landmark means (no plotting to keep dependencies minimal).

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <limits>

static double wrap_angle(double a) {
  while (a > M_PI)  a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

struct Pose2 {
  double x, y, th;
};

struct MeasRB {
  double r, b; // range, bearing
};

static MeasRB h_rb(const Pose2& x, const Eigen::Vector2d& m) {
  double dx = m(0) - x.x;
  double dy = m(1) - x.y;
  double r = std::sqrt(dx*dx + dy*dy);
  double b = wrap_angle(std::atan2(dy, dx) - x.th);
  return {r, b};
}

static Eigen::Matrix2d H_rb_wrt_m(const Pose2& x, const Eigen::Vector2d& m) {
  double dx = m(0) - x.x;
  double dy = m(1) - x.y;
  double q  = dx*dx + dy*dy;
  double r  = std::sqrt(q);
  if (r < 1e-9) { r = 1e-9; q = r*r; }
  Eigen::Matrix2d H;
  H << dx/r, dy/r,
      -dy/q,  dx/q;
  return H;
}

static Eigen::Vector2d inv_h_rb(const Pose2& x, const MeasRB& z) {
  double ang = x.th + z.b;
  return Eigen::Vector2d(x.x + z.r*std::cos(ang), x.y + z.r*std::sin(ang));
}

struct Landmark {
  Eigen::Vector2d mu;
  Eigen::Matrix2d P;
  bool active;
};

struct Mapper {
  Eigen::Matrix2d R;
  double gate; // chi2 gate (2 dof)
  std::vector<Landmark> L;

  explicit Mapper(const Eigen::Matrix2d& R_in, double gate_chi2=9.21) : R(R_in), gate(gate_chi2) {}

  void add_landmark(const Pose2& x, const MeasRB& z) {
    Landmark lm;
    lm.mu = inv_h_rb(x, z);
    lm.P = Eigen::Matrix2d::Identity() * (1.5*1.5);
    lm.active = true;
    L.push_back(lm);
  }

  int associate(const Pose2& x, const MeasRB& z, double& best_d2) const {
    if (L.empty()) { best_d2 = std::numeric_limits<double>::infinity(); return -1; }
    int best_j = -1;
    best_d2 = std::numeric_limits<double>::infinity();

    for (int j=0; j<(int)L.size(); ++j) {
      if (!L[j].active) continue;
      const auto& lm = L[j];
      Eigen::Matrix2d H = H_rb_wrt_m(x, lm.mu);
      MeasRB zhat = h_rb(x, lm.mu);
      Eigen::Vector2d y(z.r - zhat.r, wrap_angle(z.b - zhat.b));
      Eigen::Matrix2d S = H*lm.P*H.transpose() + R;
      Eigen::Matrix2d Sinv = S.inverse();
      double d2 = y.transpose() * Sinv * y;
      if (d2 < best_d2) { best_d2 = d2; best_j = j; }
    }
    if (best_d2 <= gate) return best_j;
    return -1;
  }

  void update(const Pose2& x, const MeasRB& z, int j) {
    auto& lm = L[j];
    Eigen::Matrix2d H = H_rb_wrt_m(x, lm.mu);
    MeasRB zhat = h_rb(x, lm.mu);
    Eigen::Vector2d y(z.r - zhat.r, wrap_angle(z.b - zhat.b));
    Eigen::Matrix2d S = H*lm.P*H.transpose() + R;
    Eigen::Matrix2d K = lm.P * H.transpose() * S.inverse();
    lm.mu = lm.mu + K*y;
    lm.P  = (Eigen::Matrix2d::Identity() - K*H) * lm.P;
    lm.P  = 0.5*(lm.P + lm.P.transpose());
  }
};

int main() {
  std::mt19937 rng(7);
  std::normal_distribution<double> n01(0.0, 1.0);

  // True landmarks
  std::vector<Eigen::Vector2d> Mtrue = {
    {-6.0,  5.0}, {-2.0, -4.0}, { 4.0,  6.0},
    { 7.0, -2.0}, { 1.0,  1.5}, {-7.0, -6.0}
  };

  // Trajectory (known)
  const int T = 180;
  std::vector<Pose2> X; X.reserve(T);
  for (int k=0; k<T; ++k) {
    double t = 2.0*M_PI * (double)k/(double)(T-1);
    double px = 2.5*std::cos(t) + 0.5*std::cos(3.0*t);
    double py = 2.0*std::sin(t);
    // crude heading from derivative approximation
    double t2 = 2.0*M_PI * (double)std::min(k+1,T-1)/(double)(T-1);
    double px2 = 2.5*std::cos(t2) + 0.5*std::cos(3.0*t2);
    double py2 = 2.0*std::sin(t2);
    double th = wrap_angle(std::atan2(py2-py, px2-px));
    X.push_back({px, py, th});
  }

  // Sensor
  double r_max = 9.0;
  double sigma_r = 0.15;
  double sigma_b = 2.0 * M_PI / 180.0;
  Eigen::Matrix2d R;
  R << sigma_r*sigma_r, 0.0,
       0.0, sigma_b*sigma_b;

  Mapper mapper(R, 9.21);

  for (int k=0; k<T; ++k) {
    for (const auto& m : Mtrue) {
      MeasRB z = h_rb(X[k], m);
      if (z.r <= r_max) {
        // noisy measurement
        z.r += sigma_r * n01(rng);
        z.b = wrap_angle(z.b + sigma_b * n01(rng));

        double best_d2;
        int j = mapper.associate(X[k], z, best_d2);
        if (j < 0) mapper.add_landmark(X[k], z);
        else mapper.update(X[k], z, j);
      }
    }
  }

  std::cout << "Estimated landmarks (mu):\n";
  for (size_t j=0; j<mapper.L.size(); ++j) {
    std::cout << j << ": " << mapper.L[j].mu.transpose() << "\n";
  }
  return 0;
}
