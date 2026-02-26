// Chapter 9 — Mapping Representations for Mobile Robots
// Lesson 5 (Lab): Build a 2D Occupancy Grid from LiDAR
//
// Self-contained C++17 program:
// - Simulates a robot trajectory and 2D LiDAR in a 2D world with circular obstacles + boundary walls
// - Builds occupancy grid using inverse sensor model + log-odds updates + Bresenham ray traversal
// - Writes result as a PGM image (occupancy probability) so you can view it with any image viewer
//
// Build (Linux/macOS):
//   g++ -O2 -std=c++17 Chapter9_Lesson5.cpp -o ogm
//   ./ogm
//
// Build (Windows, MSVC):
//   cl /O2 /std:c++17 Chapter9_Lesson5.cpp
//   Chapter9_Lesson5.exe

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

struct Pose2D {
  double x, y, theta;
};

struct Circle {
  double cx, cy, r;
};

static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

static inline double logit(double p) {
  p = clamp(p, 1e-9, 1.0 - 1e-9);
  return std::log(p / (1.0 - p));
}

// Ray p + t d with t >= 0, ||d||=1
static bool rayCircleIntersection(double px, double py, double dx, double dy, const Circle& c, double& t_out) {
  const double ox = px - c.cx;
  const double oy = py - c.cy;
  const double b = 2.0 * (ox * dx + oy * dy);
  const double cterm = ox * ox + oy * oy - c.r * c.r;
  const double disc = b * b - 4.0 * cterm;
  if (disc < 0.0) return false;
  const double s = std::sqrt(disc);
  const double t1 = (-b - s) / 2.0;
  const double t2 = (-b + s) / 2.0;
  double best = 1e300;
  bool ok = false;
  if (t1 >= 0.0) { best = std::min(best, t1); ok = true; }
  if (t2 >= 0.0) { best = std::min(best, t2); ok = true; }
  if (!ok) return false;
  t_out = best;
  return true;
}

static bool rayAABBIntersection(double px, double py, double dx, double dy,
                                double xmin, double xmax, double ymin, double ymax,
                                double& t_out) {
  double tmin = -1e300, tmax = 1e300;

  if (std::abs(dx) < 1e-12) {
    if (px < xmin || px > xmax) return false;
  } else {
    const double tx1 = (xmin - px) / dx;
    const double tx2 = (xmax - px) / dx;
    tmin = std::max(tmin, std::min(tx1, tx2));
    tmax = std::min(tmax, std::max(tx1, tx2));
  }

  if (std::abs(dy) < 1e-12) {
    if (py < ymin || py > ymax) return false;
  } else {
    const double ty1 = (ymin - py) / dy;
    const double ty2 = (ymax - py) / dy;
    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));
  }

  if (tmax < 0.0 || tmin > tmax) return false;
  const double t = (tmin >= 0.0) ? tmin : tmax;
  if (t < 0.0) return false;
  t_out = t;
  return true;
}

struct OccupancyGrid {
  double res;
  double origin_x, origin_y;
  int w, h;
  double l0, lmin, lmax;
  std::vector<double> logodds; // row-major: j*w + i

  OccupancyGrid(double width_m, double height_m, double res_, double ox, double oy,
                double p0=0.5, double lmin_=-8.0, double lmax_=8.0)
      : res(res_), origin_x(ox), origin_y(oy),
        w((int)std::ceil(width_m / res_)), h((int)std::ceil(height_m / res_)),
        l0(logit(p0)), lmin(lmin_), lmax(lmax_), logodds((size_t)w * (size_t)h, logit(p0)) {}

  bool worldToGrid(double x, double y, int& i, int& j) const {
    i = (int)std::floor((x - origin_x) / res);
    j = (int)std::floor((y - origin_y) / res);
    if (0 <= i && i < w && 0 <= j && j < h) return true;
    return false;
  }

  double& at(int i, int j) { return logodds[(size_t)j * (size_t)w + (size_t)i]; }
  const double& at(int i, int j) const { return logodds[(size_t)j * (size_t)w + (size_t)i]; }

  static std::vector<std::pair<int,int>> bresenham(int i0, int j0, int i1, int j1) {
    std::vector<std::pair<int,int>> cells;
    int di = std::abs(i1 - i0);
    int dj = std::abs(j1 - j0);
    int si = (i0 < i1) ? 1 : -1;
    int sj = (j0 < j1) ? 1 : -1;
    int err = di - dj;
    int i = i0, j = j0;
    while (true) {
      cells.emplace_back(i, j);
      if (i == i1 && j == j1) break;
      int e2 = 2 * err;
      if (e2 > -dj) { err -= dj; i += si; }
      if (e2 <  di) { err += di; j += sj; }
    }
    return cells;
  }

  void updateRay(const Pose2D& pose, double angle_body, double r, double zmax,
                 double l_occ, double l_free, double alpha=0.2) {
    int i0, j0;
    if (!worldToGrid(pose.x, pose.y, i0, j0)) return;

    const double ang = pose.theta + angle_body;
    const double ex = pose.x + r * std::cos(ang);
    const double ey = pose.y + r * std::sin(ang);

    int i1, j1;
    if (!worldToGrid(ex, ey, i1, j1)) return;

    auto cells = bresenham(i0, j0, i1, j1);
    if (cells.size() <= 1) return;

    const bool hit = (r < (zmax - 0.5 * alpha));
    const int last = (int)cells.size() - 1;

    // free cells: exclude endpoint if hit; else include all (except robot cell)
    const int free_end = hit ? (last - 1) : last;
    for (int k = 1; k <= free_end; ++k) {
      const auto [i, j] = cells[(size_t)k];
      at(i, j) = clamp(at(i, j) + (l_free - l0), lmin, lmax);
    }

    if (hit) {
      const auto [ie, je] = cells[(size_t)last];
      at(ie, je) = clamp(at(ie, je) + (l_occ - l0), lmin, lmax);
    }
  }

  void writePGM(const std::string& path) const {
    // p = logistic(l) in [0,1], map to [0..255]
    std::ofstream f(path, std::ios::binary);
    if (!f) {
      std::cerr << "Failed to open " << path << "\n";
      return;
    }
    f << "P5\n" << w << " " << h << "\n255\n";
    for (int j = 0; j < h; ++j) {
      for (int i = 0; i < w; ++i) {
        const double l = at(i, j);
        const double p = 1.0 / (1.0 + std::exp(-l));
        const uint8_t v = (uint8_t)clamp(std::round(p * 255.0), 0.0, 255.0);
        f.write((const char*)&v, 1);
      }
    }
  }
};

int main() {
  // World (meters)
  const double xmin = -10.0, xmax = 10.0, ymin = -10.0, ymax = 10.0;
  std::vector<Circle> circles = {
      {-3.0,  2.0, 1.2},
      { 2.5, -1.0, 1.0},
      { 4.0,  4.0, 1.5},
      {-4.5, -4.0, 1.0},
  };

  // LiDAR
  const int n_beams = 360;
  const double zmax = 8.0;

  // Grid
  const double res = 0.1;
  OccupancyGrid grid(/*width=*/20.0, /*height=*/20.0, res, /*origin_x=*/-10.0, /*origin_y=*/-10.0,
                     /*p0=*/0.5, /*lmin=*/-8.0, /*lmax=*/8.0);

  // Inverse sensor model params
  const double p_occ = 0.70, p_free = 0.30;
  const double l_occ = logit(p_occ), l_free = logit(p_free);
  const double alpha = 0.2;

  // Randomness
  std::mt19937 rng(7);
  std::normal_distribution<double> noise_r(0.0, 0.02);

  // Trajectory
  const int T = 220;
  std::vector<Pose2D> poses;
  poses.reserve(T);
  for (int t = 0; t < T; ++t) {
    const double ang = 2.0 * M_PI * (double)t / (double)T;
    const double x = 6.0 * std::cos(ang);
    const double y = 6.0 * std::sin(ang);
    const double theta = ang + M_PI / 2.0;
    poses.push_back({x, y, theta});
  }

  // Mapping
  for (const auto& pose : poses) {
    for (int k = 0; k < n_beams; ++k) {
      const double a_body = -M_PI + (2.0 * M_PI) * (double)k / (double)n_beams;
      const double ang = pose.theta + a_body;
      const double dx = std::cos(ang);
      const double dy = std::sin(ang);

      // find nearest hit among walls and circles
      double best = 1e300;
      double t_wall;
      if (rayAABBIntersection(pose.x, pose.y, dx, dy, xmin, xmax, ymin, ymax, t_wall)) {
        best = std::min(best, t_wall);
      }
      for (const auto& c : circles) {
        double t;
        if (rayCircleIntersection(pose.x, pose.y, dx, dy, c, t)) {
          best = std::min(best, t);
        }
      }

      double r = zmax;
      if (best < 1e200 && best <= zmax) {
        r = clamp(best + noise_r(rng), 0.0, zmax);
      }
      grid.updateRay(pose, a_body, r, zmax, l_occ, l_free, alpha);
    }
  }

  grid.writePGM("Chapter9_Lesson5_map.pgm");
  std::cout << "Wrote occupancy probability PGM: Chapter9_Lesson5_map.pgm\n";
  std::cout << "Tip: open the PGM with an image viewer, or convert it with ImageMagick.\n";
  return 0;
}
