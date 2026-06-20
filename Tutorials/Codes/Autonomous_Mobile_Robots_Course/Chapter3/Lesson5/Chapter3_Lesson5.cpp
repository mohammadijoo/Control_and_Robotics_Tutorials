// Chapter3_Lesson5.cpp
/*
Autonomous Mobile Robots (Control Engineering major)
Chapter 3 — Nonholonomic Motion and Feasibility for AMR
Lesson 5 — Feasibility Checks for Candidate Paths

Self-contained C++17 feasibility checker for a car-like robot.
Checks:
  - curvature/steering bounds
  - steering-rate bound (via curvature derivative and speed profile)
  - speed & longitudinal acceleration bounds (forward-backward pass)
  - collision clearance on occupancy grid (disk approximation) using multi-source Dijkstra

Build (example):
  g++ -O2 -std=c++17 Chapter3_Lesson5.cpp -o feas

Robotics ecosystem references (not required here):
  - ROS2 rclcpp, nav_msgs/Path (path ingestion)
  - nav2_costmap_2d (costmap), OMPL (planning), FCL (collision)

Author: course content generator
*/

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

struct Limits {
  double wheelbase_m = 0.33;
  double delta_max_rad = 0.45;
  double delta_dot_max_rad_s = 0.75;
  double v_max_m_s = 1.2;
  double a_long_max_m_s2 = 0.8;
  double a_lat_max_m_s2 = 1.5;
  double robot_radius_m = 0.25;
  double clearance_margin_m = 0.05;
};

struct Point2 {
  double x{0}, y{0};
};

struct Report {
  bool ok{false};
  std::string reason;
  double path_length_m{0};
  double kappa_abs_max{0};
  double kappa_max{0};
  double v_min{0};
  double v_max{0};
  double delta_dot_abs_max{0};
  double total_time_s{0};
};

static double norm2(const Point2& a) { return std::sqrt(a.x * a.x + a.y * a.y); }
static Point2 sub(const Point2& a, const Point2& b) { return {a.x - b.x, a.y - b.y}; }
static double cross2(const Point2& a, const Point2& b) { return a.x * b.y - a.y * b.x; }

static std::vector<double> arcLength(const std::vector<Point2>& xy) {
  std::vector<double> s(xy.size(), 0.0);
  for (size_t i = 1; i < xy.size(); ++i) {
    Point2 d = sub(xy[i], xy[i - 1]);
    s[i] = s[i - 1] + norm2(d);
  }
  return s;
}

// 3-point signed curvature estimate
static std::vector<double> curvatureDiscrete(const std::vector<Point2>& xy) {
  const size_t n = xy.size();
  std::vector<double> kappa(n, 0.0);
  const double eps = 1e-12;
  for (size_t i = 1; i + 1 < n; ++i) {
    Point2 p0 = xy[i - 1], p1 = xy[i], p2 = xy[i + 1];
    Point2 a = sub(p1, p0);
    Point2 b = sub(p2, p1);
    Point2 c = sub(p2, p0);
    double la = norm2(a), lb = norm2(b), lc = norm2(c);
    double area2 = cross2(a, c);
    double denom = la * lb * lc + eps;
    kappa[i] = 2.0 * area2 / denom;
  }
  if (n >= 3) {
    kappa[0] = kappa[1];
    kappa[n - 1] = kappa[n - 2];
  }
  return kappa;
}

static std::vector<double> dkappaDs(const std::vector<double>& kappa, const std::vector<double>& s) {
  const size_t n = kappa.size();
  std::vector<double> dk(n, 0.0);
  const double eps = 1e-12;
  if (n < 3) return dk;
  for (size_t i = 1; i + 1 < n; ++i) {
    double denom = (s[i + 1] - s[i - 1]) + eps;
    dk[i] = (kappa[i + 1] - kappa[i - 1]) / denom;
  }
  dk[0] = (kappa[1] - kappa[0]) / ((s[1] - s[0]) + eps);
  dk[n - 1] = (kappa[n - 1] - kappa[n - 2]) / ((s[n - 1] - s[n - 2]) + eps);
  return dk;
}

// forward-backward time scaling with lateral accel bound
static void speedProfile(
    const std::vector<double>& s,
    const std::vector<double>& kappa,
    const Limits& lim,
    std::vector<double>& v_out,
    std::vector<double>& dt_out) {

  const size_t n = s.size();
  v_out.assign(n, lim.v_max_m_s);
  dt_out.assign(n > 0 ? n - 1 : 0, 0.0);

  // per-point speed cap from lateral accel
  std::vector<double> vcap(n, lim.v_max_m_s);
  for (size_t i = 0; i < n; ++i) {
    double ak = std::fabs(kappa[i]);
    if (ak > 1e-9) {
      double vlat = std::sqrt(std::max(lim.a_lat_max_m_s2 / ak, 0.0));
      if (vlat < vcap[i]) vcap[i] = vlat;
    }
  }

  // w = v^2
  std::vector<double> w(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    double v0 = std::min(vcap[i], lim.v_max_m_s);
    w[i] = v0 * v0;
  }

  // forward (accelerate)
  for (size_t i = 0; i + 1 < n; ++i) {
    double ds = s[i + 1] - s[i];
    double w_next = w[i] + 2.0 * lim.a_long_max_m_s2 * std::max(ds, 0.0);
    if (w[i + 1] > w_next) w[i + 1] = w_next;
  }
  // backward (decelerate)
  for (size_t i = n; i-- > 1;) {
    double ds = s[i] - s[i - 1];
    double w_prev = w[i] + 2.0 * lim.a_long_max_m_s2 * std::max(ds, 0.0);
    if (w[i - 1] > w_prev) w[i - 1] = w_prev;
  }

  for (size_t i = 0; i < n; ++i) v_out[i] = std::sqrt(std::max(w[i], 0.0));

  const double eps = 1e-12;
  for (size_t i = 0; i + 1 < n; ++i) {
    double ds = s[i + 1] - s[i];
    double vavg = 0.5 * (v_out[i] + v_out[i + 1]);
    dt_out[i] = ds / (vavg + eps);
  }
}

// Grid distance to obstacles via multi-source Dijkstra (8-connected)
static std::vector<double> distanceToObstacles(
    const std::vector<uint8_t>& occ, int H, int W, double res_m) {
  const double INF = 1e18;
  std::vector<double> dist(H * W, INF);

  struct Node {
    double d;
    int idx;
    bool operator>(const Node& other) const { return d > other.d; }
  };

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

  auto push_if = [&](int idx) {
    if (occ[idx] == 1) {
      dist[idx] = 0.0;
      pq.push({0.0, idx});
    }
  };

  for (int i = 0; i < H * W; ++i) push_if(i);

  const int dx[8] = {1, -1, 0, 0,  1,  1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1,  1, -1};
  const double w[8] = {1, 1, 1, 1, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};

  while (!pq.empty()) {
    Node cur = pq.top();
    pq.pop();
    if (cur.d > dist[cur.idx]) continue;
    int y = cur.idx / W;
    int x = cur.idx % W;
    for (int k = 0; k < 8; ++k) {
      int nx = x + dx[k];
      int ny = y + dy[k];
      if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
      int nidx = ny * W + nx;
      double nd = cur.d + w[k] * res_m;
      if (nd < dist[nidx]) {
        dist[nidx] = nd;
        pq.push({nd, nidx});
      }
    }
  }
  return dist;
}

static Report checkFeasible(
    const std::vector<Point2>& xy,
    const std::vector<uint8_t>* occ, int H, int W,
    double res_m, Point2 origin,
    const Limits& lim) {

  Report rep;
  if (xy.size() < 3) {
    rep.ok = false;
    rep.reason = "invalid_path";
    return rep;
  }

  std::vector<double> s = arcLength(xy);
  std::vector<double> kappa = curvatureDiscrete(xy);
  std::vector<double> dkds = dkappaDs(kappa, s);

  double kappa_max = std::tan(lim.delta_max_rad) / lim.wheelbase_m;
  double kappa_abs_max = 0.0;
  for (double ki : kappa) kappa_abs_max = std::max(kappa_abs_max, std::fabs(ki));
  rep.kappa_abs_max = kappa_abs_max;
  rep.kappa_max = kappa_max;
  rep.path_length_m = s.back();

  if (kappa_abs_max > kappa_max + 1e-9) {
    rep.ok = false;
    rep.reason = "curvature_limit_violation";
    return rep;
  }

  // speed profile and time
  std::vector<double> v, dt;
  speedProfile(s, kappa, lim, v, dt);
  rep.v_min = *std::min_element(v.begin(), v.end());
  rep.v_max = *std::max_element(v.begin(), v.end());
  double total_t = 0.0;
  for (double ti : dt) total_t += ti;
  rep.total_time_s = total_t;

  // steering rate check
  double delta_dot_abs_max = 0.0;
  for (size_t i = 0; i < xy.size(); ++i) {
    double delta = std::atan(lim.wheelbase_m * kappa[i]);
    double kappa_dot = v[i] * dkds[i];
    double delta_dot = lim.wheelbase_m * std::pow(std::cos(delta), 2.0) * kappa_dot;
    delta_dot_abs_max = std::max(delta_dot_abs_max, std::fabs(delta_dot));
  }
  rep.delta_dot_abs_max = delta_dot_abs_max;

  if (delta_dot_abs_max > lim.delta_dot_max_rad_s + 1e-9) {
    rep.ok = false;
    rep.reason = "steering_rate_violation";
    return rep;
  }

  // collision check
  if (occ) {
    std::vector<double> dist = distanceToObstacles(*occ, H, W, res_m);
    const double needed = lim.robot_radius_m + lim.clearance_margin_m;
    for (size_t i = 0; i < xy.size(); ++i) {
      int gx = static_cast<int>(std::floor((xy[i].x - origin.x) / res_m));
      int gy = static_cast<int>(std::floor((xy[i].y - origin.y) / res_m));
      if (gx < 0 || gx >= W || gy < 0 || gy >= H) {
        rep.ok = false;
        rep.reason = "path_outside_grid";
        return rep;
      }
      double c = dist[gy * W + gx];
      if (c < needed) {
        rep.ok = false;
        rep.reason = "collision_violation";
        return rep;
      }
    }
  }

  rep.ok = true;
  rep.reason = "ok";
  return rep;
}

int main() {
  // Demo: arc path
  std::vector<Point2> path;
  const int N = 200;
  const double R = 4.0;
  for (int i = 0; i < N; ++i) {
    double t = static_cast<double>(i) / (N - 1);
    double ang = 0.6 * t;
    path.push_back({R * std::sin(ang), R * (1.0 - std::cos(ang))});
  }

  // Demo grid: 10m x 10m at 0.05m resolution, one obstacle patch
  double res = 0.05;
  int W = static_cast<int>(10.0 / res);
  int H = static_cast<int>(10.0 / res);
  std::vector<uint8_t> occ(H * W, 0);

  auto idx = [&](int y, int x) { return y * W + x; };
  int ox = static_cast<int>(2.0 / res);
  int oy = static_cast<int>(1.0 / res);
  for (int y = std::max(0, oy - 3); y < std::min(H, oy + 3); ++y)
    for (int x = std::max(0, ox - 3); x < std::min(W, ox + 3); ++x)
      occ[idx(y, x)] = 1;

  Limits lim;
  Point2 origin{0.0, 0.0};

  Report rep = checkFeasible(path, &occ, H, W, res, origin, lim);
  std::cout << "ok=" << (rep.ok ? "true" : "false") << " reason=" << rep.reason << "\n";
  std::cout << "L=" << rep.path_length_m << " kappa_abs_max=" << rep.kappa_abs_max
            << " kappa_max=" << rep.kappa_max << "\n";
  std::cout << "v_min=" << rep.v_min << " v_max=" << rep.v_max
            << " delta_dot_abs_max=" << rep.delta_dot_abs_max
            << " T=" << rep.total_time_s << "\n";
  return 0;
}
