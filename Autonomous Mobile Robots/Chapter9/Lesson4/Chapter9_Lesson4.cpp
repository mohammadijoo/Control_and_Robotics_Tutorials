// Chapter9_Lesson4.cpp
// Elevation + traversability maps (outdoor AMR) — minimal C++ implementation (no external deps).
// Output: CSV files for elevation and traversability.

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

struct Point3 {
  double x, y, z;
};

static double sigmoid(double x) {
  return 1.0 / (1.0 + std::exp(-x));
}

class ElevationTraversabilityMap {
 public:
  ElevationTraversabilityMap(double x_min, double x_max, double y_min, double y_max, double res,
                             double sigma0, double meas_sigma, double gate_k)
      : x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max), res_(res),
        R_(meas_sigma * meas_sigma), gate_k_(gate_k) {
    nx_ = static_cast<int>(std::ceil((x_max_ - x_min_) / res_));
    ny_ = static_cast<int>(std::ceil((y_max_ - y_min_) / res_));
    const int N = nx_ * ny_;
    mu_.assign(N, 0.0);
    sigma2_.assign(N, sigma0 * sigma0);
    count_.assign(N, 0);
  }

  bool updatePoint(double x, double y, double z) {
    int ix, iy;
    if (!toIndex(x, y, ix, iy)) return false;
    const int k = idx(ix, iy);

    const double mu = mu_[k];
    const double s2 = sigma2_[k];

    const double nu = z - mu;
    const double S = s2 + R_;

    if (count_[k] > 0 && std::abs(nu) > gate_k_ * std::sqrt(S)) {
      return false;
    }

    const double K = s2 / S;
    mu_[k] = mu + K * nu;
    sigma2_[k] = (1.0 - K) * s2;
    count_[k] += 1;
    return true;
  }

  int updatePointCloud(const std::vector<Point3>& pts) {
    int acc = 0;
    for (const auto& p : pts) acc += static_cast<int>(updatePoint(p.x, p.y, p.z));
    return acc;
  }

  // Compute slope/roughness/step using neighborhood finite differences.
  void computeFeatures(std::vector<double>& slope, std::vector<double>& rough, std::vector<double>& step) const {
    slope.assign(nx_ * ny_, 0.0);
    rough.assign(nx_ * ny_, 0.0);
    step.assign(nx_ * ny_, 0.0);

    for (int i = 1; i < nx_ - 1; ++i) {
      for (int j = 1; j < ny_ - 1; ++j) {
        const int k0 = idx(i, j);
        if (count_[k0] == 0) continue;

        const double dzdx = (mu_[idx(i + 1, j)] - mu_[idx(i - 1, j)]) / (2.0 * res_);
        const double dzdy = (mu_[idx(i, j + 1)] - mu_[idx(i, j - 1)]) / (2.0 * res_);
        slope[k0] = std::atan(std::sqrt(dzdx * dzdx + dzdy * dzdy));

        // neighborhood stats (3x3)
        double m = 0.0;
        double m2 = 0.0;
        int n = 0;
        double maxdiff = 0.0;
        for (int di = -1; di <= 1; ++di) {
          for (int dj = -1; dj <= 1; ++dj) {
            const double v = mu_[idx(i + di, j + dj)];
            m += v;
            m2 += v * v;
            n += 1;
            maxdiff = std::max(maxdiff, std::abs(v - mu_[k0]));
          }
        }
        m /= static_cast<double>(n);
        const double var = std::max(0.0, m2 / static_cast<double>(n) - m * m);
        rough[k0] = std::sqrt(var);
        step[k0] = maxdiff;
      }
    }
  }

  std::vector<double> traversabilityCost(const std::vector<double>& slope,
                                         const std::vector<double>& rough,
                                         const std::vector<double>& step,
                                         double slope_ref = 0.35,
                                         double rough_ref = 0.08,
                                         double step_ref  = 0.12,
                                         double w_s = 3.0, double w_r = 2.0, double w_d = 2.5,
                                         double bias = -3.0) const {
    std::vector<double> cost(nx_ * ny_, 0.0);
    const double inv_s = 1.0 / std::max(1e-9, slope_ref);
    const double inv_r = 1.0 / std::max(1e-9, rough_ref);
    const double inv_d = 1.0 / std::max(1e-9, step_ref);

    for (int k = 0; k < nx_ * ny_; ++k) {
      const double s = slope[k] * inv_s;
      const double r = rough[k] * inv_r;
      const double d = step[k] * inv_d;
      const double score = w_s * s + w_r * r + w_d * d + bias;
      cost[k] = sigmoid(score);
    }
    return cost;
  }

  void saveCSV(const std::string& path, const std::vector<double>& grid) const {
    std::ofstream f(path);
    if (!f) {
      std::cerr << "Failed to open " << path << "\n";
      return;
    }
    for (int j = 0; j < ny_; ++j) {
      for (int i = 0; i < nx_; ++i) {
        f << grid[idx(i, j)];
        if (i + 1 < nx_) f << ",";
      }
      f << "\n";
    }
  }

  void saveCSVInt(const std::string& path, const std::vector<int>& grid) const {
    std::ofstream f(path);
    if (!f) return;
    for (int j = 0; j < ny_; ++j) {
      for (int i = 0; i < nx_; ++i) {
        f << grid[idx(i, j)];
        if (i + 1 < nx_) f << ",";
      }
      f << "\n";
    }
  }

  const std::vector<double>& mu() const { return mu_; }
  const std::vector<double>& sigma2() const { return sigma2_; }
  const std::vector<int>& count() const { return count_; }

 private:
  bool toIndex(double x, double y, int& ix, int& iy) const {
    ix = static_cast<int>((x - x_min_) / res_);
    iy = static_cast<int>((y - y_min_) / res_);
    if (ix < 0 || ix >= nx_ || iy < 0 || iy >= ny_) return false;
    return true;
  }

  int idx(int ix, int iy) const { return ix + nx_ * iy; }

  double x_min_, x_max_, y_min_, y_max_, res_;
  int nx_, ny_;
  double R_, gate_k_;
  std::vector<double> mu_;
  std::vector<double> sigma2_;
  std::vector<int> count_;
};

static std::vector<Point3> syntheticPointCloud(int n_ground, int n_outliers, int seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> unif(-10.0, 10.0);
  std::normal_distribution<double> noise(0.0, 0.10);
  std::normal_distribution<double> noise_out(0.0, 0.05);

  std::vector<Point3> pts;
  pts.reserve(static_cast<size_t>(n_ground + n_outliers));

  auto z_true = [](double x, double y) {
    const double hill = 0.25 * std::exp(-0.08 * ((x - 2.0) * (x - 2.0) + (y + 1.0) * (y + 1.0)));
    return 0.20 * std::sin(0.35 * x) + 0.15 * std::cos(0.25 * y) + 0.03 * x + hill;
  };

  for (int i = 0; i < n_ground; ++i) {
    const double x = unif(rng);
    const double y = unif(rng);
    const double z = z_true(x, y) + noise(rng);
    pts.push_back({x, y, z});
  }

  for (int i = 0; i < n_outliers; ++i) {
    const double x = unif(rng);
    const double y = unif(rng);
    const double z = z_true(x, y) + 0.8 + noise_out(rng);
    pts.push_back({x, y, z});
  }

  std::shuffle(pts.begin(), pts.end(), rng);
  return pts;
}

int main() {
  ElevationTraversabilityMap emap(-10.0, 10.0, -10.0, 10.0, 0.20, 2.0, 0.10, 3.0);

  auto pts = syntheticPointCloud(50000, 2000, 7);
  const int accepted = emap.updatePointCloud(pts);
  std::cout << "Points accepted: " << accepted << " out of " << pts.size() << "\n";

  std::vector<double> slope, rough, step;
  emap.computeFeatures(slope, rough, step);
  auto cost = emap.traversabilityCost(slope, rough, step);

  emap.saveCSV("Chapter9_Lesson4_elevation_mu.csv", emap.mu());
  emap.saveCSV("Chapter9_Lesson4_traversability_cost.csv", cost);
  emap.saveCSVInt("Chapter9_Lesson4_fusion_count.csv", emap.count());

  std::cout << "Saved CSV files.\n";
  return 0;
}
