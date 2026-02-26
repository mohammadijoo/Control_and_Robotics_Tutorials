// Chapter17_Lesson3.cpp
// Autonomous Mobile Robots (Control Engineering) — Chapter 17, Lesson 3
// Next-Best-View (NBV) Strategies — minimal occupancy-grid expected-IG utility
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter17_Lesson3.cpp -o nbv
//
// This is a compact, dependency-free reference implementation (no ROS).
// It computes the best candidate viewpoint among random samples using:
//   U(v) = EIG(v) - lambda * dist(v, current)
//
// Notes:
// - Expected information gain is computed per cell using a binary sensor model
// - View EIG is approximated as a sum along ray casts with simple occlusion stopping

#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <tuple>
#include <vector>
#include <algorithm>

static inline double clamp(double x, double lo, double hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline double bernoulli_entropy(double p) {
  const double eps = 1e-12;
  p = clamp(p, eps, 1.0 - eps);
  return -p * std::log(p) - (1.0 - p) * std::log(1.0 - p);
}

struct SensorBinaryModel {
  double p_hit{0.85};   // P(z=occ | m=occ)
  double p_false{0.15}; // P(z=occ | m=free)

  double posterior(double p_occ, bool z_occ) const {
    double p = clamp(p_occ, 1e-12, 1.0 - 1e-12);
    double num = 0.0, den = 1.0;
    if (z_occ) {
      num = p_hit * p;
      den = p_hit * p + p_false * (1.0 - p);
    } else {
      num = (1.0 - p_hit) * p;
      den = (1.0 - p_hit) * p + (1.0 - p_false) * (1.0 - p);
    }
    return clamp(num / den, 1e-12, 1.0 - 1e-12);
  }

  double expected_posterior_entropy(double p_occ) const {
    double p = clamp(p_occ, 1e-12, 1.0 - 1e-12);
    double p_z_occ = p_hit * p + p_false * (1.0 - p);
    double p_z_free = 1.0 - p_z_occ;
    double p_post_occ = posterior(p, true);
    double p_post_free = posterior(p, false);
    return p_z_occ * bernoulli_entropy(p_post_occ) + p_z_free * bernoulli_entropy(p_post_free);
  }

  double expected_information_gain(double p_occ) const {
    return bernoulli_entropy(p_occ) - expected_posterior_entropy(p_occ);
  }
};

struct GridBelief {
  int W{0}, H{0};
  std::vector<double> p; // size H*W, p[y*W + x]

  GridBelief(int w, int h) : W(w), H(h), p(static_cast<size_t>(w*h), 0.5) {}

  inline bool in_bounds(int x, int y) const { return (0 <= x && x < W && 0 <= y && y < H); }
  inline double at(int x, int y) const { return p[static_cast<size_t>(y)*W + static_cast<size_t>(x)]; }
};

static std::vector<std::pair<int,int>> bresenham(int x0, int y0, int x1, int y1) {
  std::vector<std::pair<int,int>> pts;
  int dx = std::abs(x1 - x0);
  int dy = -std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;
  int x = x0, y = y0;
  while (true) {
    pts.push_back({x,y});
    if (x == x1 && y == y1) break;
    int e2 = 2*err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }
  }
  return pts;
}

static std::pair<int,int> ray_endpoint(int cx, int cy, double theta, double r) {
  int ex = static_cast<int>(std::llround(cx + r * std::cos(theta)));
  int ey = static_cast<int>(std::llround(cy + r * std::sin(theta)));
  return {ex, ey};
}

static double expected_ig_view(
  const GridBelief& grid,
  int cx, int cy, double heading,
  const SensorBinaryModel& sensor,
  double fov_rad = M_PI, int n_rays = 45, double max_range = 10.0,
  double occ_stop = 0.70
) {
  if (!grid.in_bounds(cx, cy)) return -1e9;

  double total = 0.0;
  for (int i = 0; i < n_rays; ++i) {
    double a = heading - 0.5*fov_rad + (static_cast<double>(i) / std::max(1, n_rays-1)) * fov_rad;
    auto [ex, ey] = ray_endpoint(cx, cy, a, max_range);
    auto line = bresenham(cx, cy, ex, ey);
    for (size_t k = 1; k < line.size(); ++k) {
      int x = line[k].first, y = line[k].second;
      if (!grid.in_bounds(x, y)) break;
      double p_occ = grid.at(x, y);
      total += sensor.expected_information_gain(p_occ);
      if (p_occ > occ_stop) break;
    }
  }
  return total;
}

static double dist_cost(int x0, int y0, int x1, int y1) {
  return std::hypot(static_cast<double>(x1-x0), static_cast<double>(y1-y0));
}

int main() {
  const int W = 30, H = 18;
  GridBelief grid(W, H);
  SensorBinaryModel sensor;

  std::mt19937 rng(7);
  std::uniform_int_distribution<int> ux(0, W-1), uy(0, H-1);
  std::uniform_real_distribution<double> uth(0.0, 2.0*M_PI);

  // Current pose (grid coordinates)
  int cx = 2, cy = 2;
  double heading = 0.0;

  const int N = 120;     // candidates
  const double lambda = 0.22;

  double bestU = -1e18;
  std::tuple<int,int,double,double,double> best = {cx,cy,heading,0.0,bestU};

  for (int i = 0; i < N; ++i) {
    int vx = ux(rng);
    int vy = uy(rng);
    double th = uth(rng);

    double ig = expected_ig_view(grid, vx, vy, th, sensor);
    double c = dist_cost(cx, cy, vx, vy);
    double u = ig - lambda * c;

    if (u > bestU) {
      bestU = u;
      best = {vx, vy, th, ig, u};
    }
  }

  auto [vx, vy, vth, ig, u] = best;
  std::cout << "NBV = (" << vx << "," << vy << "), heading=" << vth
            << ", IG=" << ig << ", U=" << u << "\n";
  std::cout << "(This demo only selects a view; a real system would plan & move, then update belief.)\n";
  return 0;
}
