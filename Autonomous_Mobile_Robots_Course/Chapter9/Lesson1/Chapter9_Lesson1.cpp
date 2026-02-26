/*
Chapter 9 - Mapping Representations for Mobile Robots
Lesson 1 - Occupancy Grid Mapping

A minimal occupancy grid mapper in C++ (no external robotics middleware).
- Stores a log-odds grid
- Uses Bresenham traversal for each ray
- Generates a synthetic "toy world" and simulated ray measurements
- Writes the resulting occupancy probabilities as an ASCII PGM image

Build (Linux/macOS):
  g++ -O2 -std=c++17 -o Chapter9_Lesson1 Chapter9_Lesson1.cpp

Run:
  ./Chapter9_Lesson1
  (produces: occupancy.pgm)
*/

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

static double clamp(double x, double a, double b) {
  return (x < a) ? a : (x > b) ? b : x;
}

static double logit(double p) {
  const double eps = 1e-12;
  p = clamp(p, eps, 1.0 - eps);
  return std::log(p / (1.0 - p));
}

static double inv_logit(double l) {
  return 1.0 / (1.0 + std::exp(-l));
}

static std::vector<std::pair<int,int>> bresenham(int x0, int y0, int x1, int y1) {
  std::vector<std::pair<int,int>> pts;
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = (x1 >= x0) ? 1 : -1;
  const int sy = (y1 >= y0) ? 1 : -1;
  int err = dx - dy;

  int x = x0, y = y0;
  while (true) {
    pts.emplace_back(x, y);
    if (x == x1 && y == y1) break;
    const int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x += sx; }
    if (e2 <  dx) { err += dx; y += sy; }
  }
  return pts;
}

struct OccupancyGrid {
  int W, H;
  double res;                // meters per cell
  double ox, oy;             // origin (world meters) of grid cell (0,0) corner
  double prior, p_occ, p_free;
  double l_min, l_max;
  double l_occ, l_free;
  std::vector<double> L;     // log-odds, row-major (y*W + x)

  OccupancyGrid(int w, int h, double resolution, double origin_x, double origin_y,
                double prior_p = 0.5, double pocc = 0.75, double pfree = 0.35)
    : W(w), H(h), res(resolution), ox(origin_x), oy(origin_y),
      prior(prior_p), p_occ(pocc), p_free(pfree),
      l_min(-10.0), l_max(10.0)
  {
    const double l0 = logit(prior);
    L.assign(static_cast<size_t>(W * H), l0);
    l_occ  = logit(p_occ)  - logit(prior);
    l_free = logit(p_free) - logit(prior);
  }

  inline bool in_bounds(int gx, int gy) const {
    return (0 <= gx && gx < W && 0 <= gy && gy < H);
  }

  inline std::pair<int,int> world_to_grid(double x, double y) const {
    const int gx = static_cast<int>(std::floor((x - ox) / res));
    const int gy = static_cast<int>(std::floor((y - oy) / res));
    return {gx, gy};
  }

  inline double& at(int gx, int gy) {
    return L[static_cast<size_t>(gy * W + gx)];
  }

  void update_ray(double x, double y, double theta, double rel_angle, double range, double max_range) {
    range = clamp(range, 0.0, max_range);
    const double a = theta + rel_angle;
    const double xe = x + range * std::cos(a);
    const double ye = y + range * std::sin(a);

    const auto [x0, y0] = world_to_grid(x, y);
    const auto [x1, y1] = world_to_grid(xe, ye);
    if (!in_bounds(x0, y0)) return;

    auto cells = bresenham(x0, y0, x1, y1);
    if (cells.empty()) return;

    const bool hit = (range < max_range - 1e-6);
    const size_t n_free = hit ? (cells.size() - 1) : cells.size();

    for (size_t i = 0; i < n_free; ++i) {
      const int cx = cells[i].first;
      const int cy = cells[i].second;
      if (in_bounds(cx, cy)) {
        at(cx, cy) = clamp(at(cx, cy) + l_free, l_min, l_max);
      }
    }
    if (hit) {
      const int cx = cells.back().first;
      const int cy = cells.back().second;
      if (in_bounds(cx, cy)) {
        at(cx, cy) = clamp(at(cx, cy) + l_occ, l_min, l_max);
      }
    }
  }

  double prob(int gx, int gy) const {
    return inv_logit(L[static_cast<size_t>(gy * W + gx)]);
  }
};

static std::vector<uint8_t> build_toy_world(int W, int H) {
  std::vector<uint8_t> occ(static_cast<size_t>(W * H), 0);
  auto idx = [W](int x, int y) { return static_cast<size_t>(y * W + x); };

  // Border walls
  for (int x = 0; x < W; ++x) {
    occ[idx(x, 0)] = 1;
    occ[idx(x, H-1)] = 1;
  }
  for (int y = 0; y < H; ++y) {
    occ[idx(0, y)] = 1;
    occ[idx(W-1, y)] = 1;
  }

  // Rectangle obstacle
  for (int y = 30; y < 50; ++y)
    for (int x = 55; x < 75; ++x)
      occ[idx(x, y)] = 1;

  // Small obstacle
  for (int y = 70; y < 80; ++y)
    for (int x = 25; x < 35; ++x)
      occ[idx(x, y)] = 1;

  return occ;
}

static double cast_ray_grid(const std::vector<uint8_t>& world, const OccupancyGrid& grid,
                            double x, double y, double theta, double rel_angle, double max_range,
                            double step) {
  const double a = theta + rel_angle;
  double dist = 0.0;

  auto idx = [grid](int gx, int gy) { return static_cast<size_t>(gy * grid.W + gx); };

  while (dist < max_range) {
    const double xt = x + dist * std::cos(a);
    const double yt = y + dist * std::sin(a);
    const auto [gx, gy] = grid.world_to_grid(xt, yt);

    if (grid.in_bounds(gx, gy)) {
      if (world[idx(gx, gy)] == 1) return dist;
    } else {
      return dist; // out-of-bounds acts like a wall
    }
    dist += step;
  }
  return max_range;
}

static void write_pgm(const std::string& path, const OccupancyGrid& grid) {
  std::ofstream out(path);
  if (!out) {
    std::cerr << "Failed to open output file: " << path << "\n";
    return;
  }
  // ASCII PGM: P2
  out << "P2\n" << grid.W << " " << grid.H << "\n255\n";
  for (int y = grid.H - 1; y >= 0; --y) { // write top row first for conventional image
    for (int x = 0; x < grid.W; ++x) {
      const double p = grid.prob(x, y);
      const int pix = static_cast<int>(std::round(255.0 * (1.0 - p))); // occupied -> dark
      out << pix << (x + 1 < grid.W ? " " : "\n");
    }
  }
}

int main() {
  const int W = 120, H = 100;
  const double res = 0.05;
  OccupancyGrid og(W, H, res, 0.0, 0.0, 0.5, 0.75, 0.35);

  const auto world = build_toy_world(W, H);

  // Trajectory
  std::vector<std::tuple<double,double,double>> traj;
  for (int k = 0; k < 25; ++k) {
    const double x = 0.8 + 0.06 * k;
    const double y = 0.9 + 0.01 * k;
    const double theta = 0.15;
    traj.emplace_back(x, y, theta);
  }

  const double fov = M_PI;   // 180 deg
  const int n_rays = 121;
  const double z_max = 3.0;
  const double step = res * 0.5;

  for (const auto& p : traj) {
    const double x = std::get<0>(p);
    const double y = std::get<1>(p);
    const double theta = std::get<2>(p);

    for (int i = 0; i < n_rays; ++i) {
      const double ang = -fov / 2.0 + (fov * i) / (n_rays - 1);
      const double z = cast_ray_grid(world, og, x, y, theta, ang, z_max, step);
      og.update_ray(x, y, theta, ang, z, z_max);
    }
  }

  write_pgm("occupancy.pgm", og);
  std::cout << "Wrote occupancy.pgm (ASCII PGM). Open with an image viewer or convert to PNG.\n";
  return 0;
}
