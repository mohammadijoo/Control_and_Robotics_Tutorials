// Chapter9_Lesson2.cpp
// Autonomous Mobile Robots — Chapter 9, Lesson 2
// Log-Odds Updates and Bayesian Cells (Occupancy Grid Mapping)
//
// Minimal, dependency-free C++ example that:
//   1) maintains a log-odds occupancy grid
//   2) applies an inverse sensor model update along range rays (Bresenham)
//   3) exports the fused occupancy map as a PGM image (viewable with many image viewers).
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter9_Lesson2.cpp -o Chapter9_Lesson2
// Run:
//   ./Chapter9_Lesson2
//
// Output: occupancy.pgm

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float logit(float p) {
  const float eps = 1e-6f;
  p = clampf(p, eps, 1.0f - eps);
  return std::log(p / (1.0f - p));
}

static inline float logistic(float l) {
  // Stable sigmoid
  if (l >= 0.0f) {
    float z = std::exp(-l);
    return 1.0f / (1.0f + z);
  } else {
    float z = std::exp(l);
    return z / (1.0f + z);
  }
}

struct Grid {
  int width = 0;
  int height = 0;
  float res = 0.05f;        // m/cell
  float origin_x = -5.5f;   // world coord of cell (0,0) center
  float origin_y = -5.5f;
  std::vector<float> log_odds; // row-major, size = width*height

  Grid(int w, int h, float resolution, float ox, float oy)
      : width(w), height(h), res(resolution), origin_x(ox), origin_y(oy),
        log_odds(static_cast<size_t>(w) * static_cast<size_t>(h), 0.0f) {}

  inline bool inBounds(int ix, int iy) const {
    return (0 <= ix && ix < width && 0 <= iy && iy < height);
  }

  inline int idx(int ix, int iy) const { return iy * width + ix; }

  inline std::pair<int, int> worldToMap(float x, float y) const {
    int ix = static_cast<int>(std::lround((x - origin_x) / res));
    int iy = static_cast<int>(std::lround((y - origin_y) / res));
    return {ix, iy};
  }

  void updateCell(int ix, int iy, float delta_l, float lmin, float lmax) {
    if (!inBounds(ix, iy)) return;
    int k = idx(ix, iy);
    log_odds[k] = clampf(log_odds[k] + delta_l, lmin, lmax);
  }
};

static std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1) {
  std::vector<std::pair<int, int>> pts;
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x1 >= x0) ? 1 : -1;
  int sy = (y1 >= y0) ? 1 : -1;

  int x = x0, y = y0;
  if (dy <= dx) {
    int err = dx / 2;
    while (x != x1) {
      pts.push_back({x, y});
      err -= dy;
      if (err < 0) {
        y += sy;
        err += dx;
      }
      x += sx;
    }
    pts.push_back({x1, y1});
  } else {
    int err = dy / 2;
    while (y != y1) {
      pts.push_back({x, y});
      err -= dx;
      if (err < 0) {
        x += sx;
        err += dy;
      }
      y += sy;
    }
    pts.push_back({x1, y1});
  }
  return pts;
}

// Synthetic raycast against circles
static float raycastCircles(float x, float y, float th, float a, float range_max,
                            const std::vector<std::tuple<float, float, float>>& circles) {
  float dx = std::cos(th + a);
  float dy = std::sin(th + a);
  float step = 0.02f;
  for (float t = 0.0f; t <= range_max; t += step) {
    float px = x + t * dx;
    float py = y + t * dy;
    for (const auto& c : circles) {
      float cx, cy, rr;
      std::tie(cx, cy, rr) = c;
      float ex = px - cx;
      float ey = py - cy;
      if (ex * ex + ey * ey <= rr * rr) {
        return std::max(0.05f, t); // return hit range (no noise here)
      }
    }
  }
  return range_max;
}

static void updateScan(Grid& g,
                       float x, float y, float th,
                       const std::vector<float>& angles,
                       const std::vector<float>& ranges,
                       float range_max,
                       float p0 = 0.5f, float p_occ = 0.7f, float p_free = 0.3f,
                       float lmin = -10.0f, float lmax = 10.0f) {
  float l0 = logit(p0);
  float l_occ = logit(p_occ) - l0;
  float l_free = logit(p_free) - l0;

  auto [sx, sy] = g.worldToMap(x, y);

  for (size_t i = 0; i < angles.size(); ++i) {
    float a = angles[i];
    float r = ranges[i];
    if (!std::isfinite(r) || r <= 0.0f) continue;
    float r_eff = std::min(r, range_max);

    float bx = x + r_eff * std::cos(th + a);
    float by = y + r_eff * std::sin(th + a);
    auto [ex, ey] = g.worldToMap(bx, by);

    auto ray = bresenham(sx, sy, ex, ey);
    if (ray.empty()) continue;

    for (size_t k = 0; k + 1 < ray.size(); ++k) {
      g.updateCell(ray[k].first, ray[k].second, l_free, lmin, lmax);
    }
    if (r < range_max - 1e-6f) {
      g.updateCell(ray.back().first, ray.back().second, l_occ, lmin, lmax);
    }
  }
}

static void savePGM(const Grid& g, const std::string& path) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    std::cerr << "Failed to open output file: " << path << "\n";
    return;
  }
  out << "P5\n" << g.width << " " << g.height << "\n255\n";

  // Convert probability to grayscale: occupied -> dark (low value), free -> bright.
  for (int iy = g.height - 1; iy >= 0; --iy) { // flip for nicer view
    for (int ix = 0; ix < g.width; ++ix) {
      float l = g.log_odds[g.idx(ix, iy)];
      float p = logistic(l);
      uint8_t v = static_cast<uint8_t>(clampf(255.0f * (1.0f - p), 0.0f, 255.0f));
      out.write(reinterpret_cast<const char*>(&v), 1);
    }
  }
}

int main() {
  Grid grid(220, 220, 0.05f, -5.5f, -5.5f);

  std::vector<std::tuple<float, float, float>> circles = {
      {1.5f, 0.5f, 0.6f},
      {-1.2f, 1.2f, 0.5f},
  };

  // Angles for a coarse scan
  std::vector<float> angles;
  const int N = 121;
  for (int i = 0; i < N; ++i) {
    float a = -static_cast<float>(M_PI) / 2.0f + (static_cast<float>(i) / (N - 1)) * static_cast<float>(M_PI);
    angles.push_back(a);
  }
  float range_max = 6.0f;

  std::vector<std::tuple<float, float, float>> poses = {
      {-2.0f, -2.0f, 0.2f},
      {0.0f, -2.5f, 0.6f},
      {2.0f, -1.5f, 1.0f},
      {2.0f, 1.0f, 1.6f},
      {0.0f, 2.5f, 2.5f},
      {-2.0f, 2.0f, -2.7f},
  };

  for (const auto& pose : poses) {
    float x, y, th;
    std::tie(x, y, th) = pose;
    std::vector<float> ranges;
    ranges.reserve(angles.size());
    for (float a : angles) {
      ranges.push_back(raycastCircles(x, y, th, a, range_max, circles));
    }
    updateScan(grid, x, y, th, angles, ranges, range_max);
  }

  savePGM(grid, "occupancy.pgm");
  std::cout << "Wrote occupancy.pgm\n";
  return 0;
}
