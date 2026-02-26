// Chapter18_Lesson2.cpp
// Traversability + terrain classification (toy implementation)
// Build: g++ -O2 -std=c++17 Chapter18_Lesson2.cpp -o trav
// Run:   ./trav --points_csv points.csv --resolution 0.25 --out_csv traversability_map.csv
//
// CSV input columns: x,y,z (no header required).
// Output CSV columns: x,y,p_trav,slope_deg,rough_m,step_m,count
//
// This is a minimal educational implementation (no ROS/PCL). For a real robot,
// you would subscribe to PointCloud2, build an elevation map (e.g., grid_map / elevation_mapping),
// and publish a traversability layer for planning.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

struct GridSpec {
  double x_min, x_max, y_min, y_max, res;
  int width() const { return static_cast<int>(std::ceil((x_max - x_min) / res)); }
  int height() const { return static_cast<int>(std::ceil((y_max - y_min) / res)); }
};

static bool read_xyz_csv(const std::string& path, std::vector<double>& xs, std::vector<double>& ys, std::vector<double>& zs) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string a, b, c;
    if (!std::getline(ss, a, ',')) continue;
    if (!std::getline(ss, b, ',')) continue;
    if (!std::getline(ss, c, ',')) continue;
    try {
      xs.push_back(std::stod(a));
      ys.push_back(std::stod(b));
      zs.push_back(std::stod(c));
    } catch (...) {
      // allow header
      continue;
    }
  }
  return !xs.empty();
}

static inline double sigmoid(double s) { return 1.0 / (1.0 + std::exp(-s)); }

static void points_to_height_map(
  const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& zs,
  const GridSpec& g, std::vector<double>& H, std::vector<int>& C
) {
  const int Nx = g.width(), Ny = g.height();
  H.assign(static_cast<std::size_t>(Nx * Ny), 0.0);
  C.assign(static_cast<std::size_t>(Nx * Ny), 0);

  for (std::size_t i = 0; i < xs.size(); ++i) {
    int ix = static_cast<int>(std::floor((xs[i] - g.x_min) / g.res));
    int iy = static_cast<int>(std::floor((ys[i] - g.y_min) / g.res));
    if (ix < 0 || ix >= Nx || iy < 0 || iy >= Ny) continue;
    const std::size_t k = static_cast<std::size_t>(iy * Nx + ix);
    H[k] += zs[i];
    C[k] += 1;
  }

  for (std::size_t k = 0; k < H.size(); ++k) {
    if (C[k] > 0) H[k] /= static_cast<double>(C[k]);
    else H[k] = std::numeric_limits<double>::quiet_NaN();
  }
}

static void fill_nans_neighborhood(std::vector<double>& H, int Nx, int Ny, int iters = 6) {
  for (int it = 0; it < iters; ++it) {
    bool any = false;
    std::vector<double> out = H;
    for (int y = 0; y < Ny; ++y) {
      for (int x = 0; x < Nx; ++x) {
        const std::size_t k = static_cast<std::size_t>(y * Nx + x);
        if (!std::isnan(H[k])) continue;
        any = true;
        double s = 0.0;
        int cnt = 0;
        for (int yy = std::max(0, y - 1); yy <= std::min(Ny - 1, y + 1); ++yy) {
          for (int xx = std::max(0, x - 1); xx <= std::min(Nx - 1, x + 1); ++xx) {
            const std::size_t kk = static_cast<std::size_t>(yy * Nx + xx);
            if (std::isnan(H[kk])) continue;
            s += H[kk];
            cnt += 1;
          }
        }
        if (cnt > 0) out[k] = s / static_cast<double>(cnt);
      }
    }
    H.swap(out);
    if (!any) break;
  }
}

static void compute_features(
  const std::vector<double>& H_in, const std::vector<int>& C,
  const GridSpec& g,
  std::vector<double>& slope_rad, std::vector<double>& rough, std::vector<double>& step
) {
  const int Nx = g.width(), Ny = g.height();
  std::vector<double> H = H_in;
  fill_nans_neighborhood(H, Nx, Ny, 6);

  slope_rad.assign(static_cast<std::size_t>(Nx * Ny), 0.0);
  rough.assign(static_cast<std::size_t>(Nx * Ny), 0.0);
  step.assign(static_cast<std::size_t>(Nx * Ny), 0.0);

  auto idx = [Nx](int x, int y) -> std::size_t { return static_cast<std::size_t>(y * Nx + x); };

  // finite differences
  for (int y = 0; y < Ny; ++y) {
    for (int x = 0; x < Nx; ++x) {
      const double hx1 = H[idx(std::min(Nx - 1, x + 1), y)];
      const double hx0 = H[idx(std::max(0, x - 1), y)];
      const double hy1 = H[idx(x, std::min(Ny - 1, y + 1))];
      const double hy0 = H[idx(x, std::max(0, y - 1))];
      const double dhdx = (hx1 - hx0) / (2.0 * g.res);
      const double dhdy = (hy1 - hy0) / (2.0 * g.res);
      const double gn = std::sqrt(dhdx * dhdx + dhdy * dhdy);
      slope_rad[idx(x, y)] = std::atan(gn);
    }
  }

  // 3x3 roughness (std) and step (max-min)
  for (int y = 0; y < Ny; ++y) {
    for (int x = 0; x < Nx; ++x) {
      double s = 0.0, s2 = 0.0;
      double mn = +std::numeric_limits<double>::infinity();
      double mx = -std::numeric_limits<double>::infinity();
      int cnt = 0;
      for (int yy = std::max(0, y - 1); yy <= std::min(Ny - 1, y + 1); ++yy) {
        for (int xx = std::max(0, x - 1); xx <= std::min(Nx - 1, x + 1); ++xx) {
          const double v = H[idx(xx, yy)];
          s += v;
          s2 += v * v;
          mn = std::min(mn, v);
          mx = std::max(mx, v);
          cnt += 1;
        }
      }
      const double mu = s / static_cast<double>(cnt);
      const double var = std::max(0.0, s2 / static_cast<double>(cnt) - mu * mu);
      rough[idx(x, y)] = std::sqrt(var);
      step[idx(x, y)] = mx - mn;
    }
  }

  // keep invalid cells as NaN in features where count==0
  for (int y = 0; y < Ny; ++y) {
    for (int x = 0; x < Nx; ++x) {
      const std::size_t k = idx(x, y);
      if (C[k] == 0) {
        slope_rad[k] = std::numeric_limits<double>::quiet_NaN();
        rough[k] = std::numeric_limits<double>::quiet_NaN();
        step[k] = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }
}

static void usage() {
  std::cout << "Usage:\n"
            << "  trav --points_csv points.csv --resolution 0.25 --out_csv traversability_map.csv\n";
}

int main(int argc, char** argv) {
  std::string points_csv;
  std::string out_csv = "traversability_map.csv";
  double res = 0.25;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--points_csv" && i + 1 < argc) points_csv = argv[++i];
    else if (a == "--out_csv" && i + 1 < argc) out_csv = argv[++i];
    else if (a == "--resolution" && i + 1 < argc) res = std::stod(argv[++i]);
    else if (a == "--help") { usage(); return 0; }
  }

  if (points_csv.empty()) {
    std::cerr << "Error: --points_csv required for this C++ demo.\n";
    usage();
    return 1;
  }

  std::vector<double> xs, ys, zs;
  if (!read_xyz_csv(points_csv, xs, ys, zs)) {
    std::cerr << "Failed to read points from " << points_csv << "\n";
    return 1;
  }

  double x_min = *std::min_element(xs.begin(), xs.end());
  double x_max = *std::max_element(xs.begin(), xs.end());
  double y_min = *std::min_element(ys.begin(), ys.end());
  double y_max = *std::max_element(ys.begin(), ys.end());

  GridSpec g{ x_min, x_max, y_min, y_max, res };

  std::vector<double> H;
  std::vector<int> C;
  points_to_height_map(xs, ys, zs, g, H, C);

  std::vector<double> slope, rough, step;
  compute_features(H, C, g, slope, rough, step);

  // A simple traversability probability model:
  // s = b + w1*slope_rad + w2*rough + w3*step
  // p = sigmoid(s)
  //
  // These weights are illustrative. In practice, learn/calibrate them from data.
  const double b = 3.0;
  const double w1 = -6.0;   // penalize slope
  const double w2 = -40.0;  // penalize roughness [m]
  const double w3 = -25.0;  // penalize step height [m]

  std::ofstream out(out_csv);
  if (!out) {
    std::cerr << "Cannot write " << out_csv << "\n";
    return 1;
  }
  out << "x,y,p_trav,slope_deg,rough_m,step_m,count\n";

  const int Nx = g.width(), Ny = g.height();
  auto idx = [Nx](int x, int y) -> std::size_t { return static_cast<std::size_t>(y * Nx + x); };

  for (int iy = 0; iy < Ny; ++iy) {
    const double yc = g.y_min + (static_cast<double>(iy) + 0.5) * g.res;
    for (int ix = 0; ix < Nx; ++ix) {
      const std::size_t k = idx(ix, iy);
      if (C[k] == 0) continue;
      const double s = b + w1 * slope[k] + w2 * rough[k] + w3 * step[k];
      const double p = sigmoid(s);
      const double slope_deg = slope[k] * 180.0 / M_PI;
      const double xc = g.x_min + (static_cast<double>(ix) + 0.5) * g.res;
      out << xc << "," << yc << "," << p << "," << slope_deg << "," << rough[k] << "," << step[k] << "," << C[k] << "\n";
    }
  }

  std::cout << "Exported: " << out_csv << "\n";
  return 0;
}
