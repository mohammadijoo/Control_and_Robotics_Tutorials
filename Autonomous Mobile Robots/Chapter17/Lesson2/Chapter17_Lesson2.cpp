/*
Chapter 17 - Lesson 2: Information Gain and Entropy Reduction
Autonomous Mobile Robots (Control Engineering)

Minimal C++17 implementation for:
- Bernoulli entropy
- Per-cell information gain for a binary sensor model
- A simple expected IG estimator for a candidate viewpoint (ray stepping)

Build:
  g++ -std=c++17 -O2 Chapter17_Lesson2.cpp -o ig_demo
Run:
  ./ig_demo
*/

#include <cmath>
#include <iostream>
#include <random>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

static double clip(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static double bernoulli_entropy(double p, double eps = 1e-12) {
  p = clip(p, eps, 1.0 - eps);
  return -p * std::log(p) - (1.0 - p) * std::log(1.0 - p);
}

static double info_gain_cell(double p, double p_hit = 0.85, double p_false = 0.15) {
  const double H_prior = bernoulli_entropy(p);

  double P_z_occ = p_hit * p + p_false * (1.0 - p);
  double P_z_free = 1.0 - P_z_occ;

  const double eps = 1e-15;
  P_z_occ = clip(P_z_occ, eps, 1.0 - eps);
  P_z_free = clip(P_z_free, eps, 1.0 - eps);

  const double p_post_z_occ = (p_hit * p) / P_z_occ;
  const double p_post_z_free = ((1.0 - p_hit) * p) / P_z_free;

  const double H_post =
      P_z_occ * bernoulli_entropy(p_post_z_occ) + P_z_free * bernoulli_entropy(p_post_z_free);

  return H_prior - H_post;
}

struct Grid {
  int width;
  int height;
  double resolution;
  double origin_x;
  double origin_y;
  std::vector<double> p; // row-major

  Grid(int w, int h, double res, double ox, double oy)
      : width(w), height(h), resolution(res), origin_x(ox), origin_y(oy), p(w * h, 0.5) {}

  bool in_bounds(int ix, int iy) const { return (0 <= ix && ix < width && 0 <= iy && iy < height); }

  std::pair<int, int> world_to_grid(double x, double y) const {
    const int ix = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int iy = static_cast<int>(std::floor((y - origin_y) / resolution));
    return {ix, iy};
  }

  double &at(int ix, int iy) { return p[iy * width + ix]; }
  double at(int ix, int iy) const { return p[iy * width + ix]; }

  double map_entropy() const {
    double s = 0.0;
    for (double prob : p) s += bernoulli_entropy(prob);
    return s;
  }
};

static std::vector<std::pair<int, int>> ray_cast_cells(const Grid &g, double x0, double y0, double theta,
                                                       double max_range, double step = -1.0) {
  if (step <= 0.0) step = 0.5 * g.resolution;

  std::vector<std::pair<int, int>> cells;
  std::set<std::pair<int, int>> seen;

  for (double t = 0.0; t <= max_range; t += step) {
    const double x = x0 + t * std::cos(theta);
    const double y = y0 + t * std::sin(theta);
    auto [ix, iy] = g.world_to_grid(x, y);
    if (!g.in_bounds(ix, iy)) break;
    std::pair<int, int> key = {ix, iy};
    if (!seen.count(key)) {
      seen.insert(key);
      cells.push_back(key);
    }
  }
  return cells;
}

static double expected_info_gain_view(const Grid &g, std::tuple<double, double, double> pose,
                                      double fov_deg = 180.0, int n_rays = 181, double max_range = 8.0,
                                      double p_hit = 0.85, double p_false = 0.15,
                                      std::pair<double, double> unknown_band = {0.4, 0.6},
                                      double occ_stop_threshold = 0.75) {
  const auto [x, y, yaw] = pose;

  const double fov = fov_deg * M_PI / 180.0;
  std::vector<double> angles;
  if (n_rays < 2) {
    angles.push_back(0.0);
  } else {
    angles.reserve(n_rays);
    for (int i = 0; i < n_rays; ++i) {
      angles.push_back((-0.5 * fov) + i * (fov / (n_rays - 1)));
    }
  }

  const double lo_u = unknown_band.first;
  const double hi_u = unknown_band.second;

  std::set<std::pair<int, int>> visited;
  double ig_total = 0.0;

  for (double a : angles) {
    const double theta = yaw + a;
    auto cells = ray_cast_cells(g, x, y, theta, max_range);

    for (auto [ix, iy] : cells) {
      std::pair<int, int> key = {ix, iy};
      if (visited.count(key)) continue;
      visited.insert(key);

      const double p = g.at(ix, iy);

      if (p >= occ_stop_threshold) break; // occlusion

      if (lo_u <= p && p <= hi_u) {
        ig_total += info_gain_cell(p, p_hit, p_false);
      }
    }
  }

  return ig_total;
}

int main() {
  Grid g(120, 120, 0.1, -6.0, -6.0);

  std::mt19937 rng(0);
  std::normal_distribution<double> free_d(0.15, 0.10);
  std::normal_distribution<double> occ_d(0.85, 0.08);
  std::uniform_int_distribution<int> ix_d(0, g.width - 1);
  std::uniform_int_distribution<int> iy_d(0, g.height - 1);

  for (int k = 0; k < 4000; ++k) {
    int ix = ix_d(rng), iy = iy_d(rng);
    g.at(ix, iy) = clip(free_d(rng), 0.01, 0.99);
  }
  for (int k = 0; k < 1200; ++k) {
    int ix = ix_d(rng), iy = iy_d(rng);
    g.at(ix, iy) = clip(occ_d(rng), 0.01, 0.99);
  }

  std::cout << "Initial map entropy (nats): " << g.map_entropy() << "
";

  std::vector<std::tuple<double, double, double>> poses = {
      {0.0, 0.0, 0.0},
      {-2.0, 1.5, 0.7},
      {2.0, -1.0, -1.2},
  };

  for (auto pose : poses) {
    double ig = expected_info_gain_view(g, pose, 180.0, 181, 6.0);
    std::cout << "Pose=(" << std::get<0>(pose) << "," << std::get<1>(pose) << "," << std::get<2>(pose)
              << ")  Expected IG≈ " << ig << " nats
";
  }

  return 0;
}
