// Chapter16_Lesson5.cpp
// Autonomous Mobile Robots — Chapter 16, Lesson 5
// Lab: Navigate Through Moving Crowds
//
// Minimal C++17 simulator (no external deps):
//   - Crowd: constant-velocity discs with small jitter
//   - Robot: unicycle
//   - Planner: sampling-based crowd-aware DWA (TTC + social discomfort)
//
// Build (example):
//   g++ -std=c++17 -O2 -o Chapter16_Lesson5 Chapter16_Lesson5.cpp
// Run:
//   ./Chapter16_Lesson5
//
// Output:
//   Writes robot trajectory to "robot_traj.csv" in the working directory.

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

struct Vec2 {
  double x{0.0}, y{0.0};
  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}
  Vec2 operator+(const Vec2& o) const { return Vec2{x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return Vec2{x - o.x, y - o.y}; }
  Vec2 operator*(double s) const { return Vec2{x * s, y * s}; }
};

static inline double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
static inline double norm(const Vec2& a) { return std::sqrt(dot(a, a)); }
static inline Vec2 rot(double phi, const Vec2& v) {
  double c = std::cos(phi), s = std::sin(phi);
  return Vec2{c * v.x - s * v.y, s * v.x + c * v.y};
}
static inline double wrap_angle(double a) {
  const double pi = 3.14159265358979323846;
  a = std::fmod(a + pi, 2.0 * pi);
  if (a < 0.0) a += 2.0 * pi;
  return a - pi;
}
static inline double clamp(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

struct DiscAgent {
  Vec2 p;
  Vec2 v;
  double r{0.25};
};

struct Robot {
  Vec2 p;
  double th{0.0};
  double v{0.0};
  double w{0.0};
  double r{0.32};
};

struct Params {
  double dt{0.1};
  double horizon{2.5};
  double v_max{1.2};
  double w_max{1.8};
  double a_v{1.0};
  double a_w{2.5};

  double w_goal{5.0};
  double w_clear{2.0};
  double w_ttc{4.0};
  double w_soc{2.0};
  double w_vel{0.5};
  double w_turn{0.1};
  double w_smooth{0.2};

  double sigma_front{0.9};
  double sigma_side{0.6};
  double sigma_back{0.5};
};

static double ttc_discs(const Vec2& p_rel, const Vec2& v_rel, double R) {
  const double eps = 1e-9;
  double a = dot(v_rel, v_rel);
  double b = 2.0 * dot(p_rel, v_rel);
  double c = dot(p_rel, p_rel) - R * R;

  if (c <= 0.0) return 0.0;
  if (a <= eps) return std::numeric_limits<double>::infinity();

  double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) return std::numeric_limits<double>::infinity();

  double s = std::sqrt(disc);
  double t1 = (-b - s) / (2.0 * a);
  double t2 = (-b + s) / (2.0 * a);

  if (t1 >= 0.0) return t1;
  if (t2 >= 0.0) return t2;
  return std::numeric_limits<double>::infinity();
}

static double social_cost(const Vec2& rp, const std::vector<DiscAgent>& humans, const Params& P) {
  double c = 0.0;
  for (const auto& h : humans) {
    double vnorm = norm(h.v);
    double phi = (vnorm < 1e-6) ? 0.0 : std::atan2(h.v.y, h.v.x);

    // world->human frame rotation is transpose, equivalent to rot(-phi)
    Vec2 rel = rp - h.p;
    Vec2 rel_h = rot(-phi, rel);
    double dx = rel_h.x, dy = rel_h.y;

    double sx = (dx >= 0.0) ? P.sigma_front : P.sigma_back;
    double sy = P.sigma_side;
    double q = 0.5 * ((dx / sx) * (dx / sx) + (dy / sy) * (dy / sy));
    c += std::exp(-q);
  }
  return c;
}

static void simulate_unicycle(Vec2& p, double& th, double v, double w, double dt) {
  p.x += v * std::cos(th) * dt;
  p.y += v * std::sin(th) * dt;
  th = wrap_angle(th + w * dt);
}

static std::vector<DiscAgent> make_crossing_crowd(int n_per_stream, double speed, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> uni_y(-2.0, 2.0);
  std::uniform_real_distribution<double> uni_xL(-7.0, -3.5);
  std::uniform_real_distribution<double> uni_x(-2.0, 2.0);
  std::uniform_real_distribution<double> uni_yB(-7.0, -3.5);

  std::vector<DiscAgent> humans;
  humans.reserve(2 * n_per_stream);

  for (int i = 0; i < n_per_stream; ++i) {
    DiscAgent h;
    h.p = Vec2{uni_xL(rng), uni_y(rng)};
    h.v = Vec2{speed, 0.0};
    h.r = 0.25;
    humans.push_back(h);
  }
  for (int i = 0; i < n_per_stream; ++i) {
    DiscAgent h;
    h.p = Vec2{uni_x(rng), uni_yB(rng)};
    h.v = Vec2{0.0, speed};
    h.r = 0.25;
    humans.push_back(h);
  }
  return humans;
}

static void update_humans(std::vector<DiscAgent>& humans, double dt, double arena, double noise_std, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, noise_std);

  for (auto& h : humans) {
    if (noise_std > 0.0) {
      h.v.x += noise(rng);
      h.v.y += noise(rng);
      double sp = norm(h.v);
      if (sp > 1e-6) {
        double sp2 = std::min(sp, 1.5);
        h.v.x = h.v.x / sp * sp2;
        h.v.y = h.v.y / sp * sp2;
      }
    }
    h.p = h.p + h.v * dt;

    if (h.p.x < -arena) { h.p.x = -arena; h.v.x = std::fabs(h.v.x); }
    if (h.p.x >  arena) { h.p.x =  arena; h.v.x = -std::fabs(h.v.x); }
    if (h.p.y < -arena) { h.p.y = -arena; h.v.y = std::fabs(h.v.y); }
    if (h.p.y >  arena) { h.p.y =  arena; h.v.y = -std::fabs(h.v.y); }
  }
}

static std::pair<double, double> plan_control(const Robot& robot, const Vec2& goal, const std::vector<DiscAgent>& humans, const Params& P) {
  const double dt = P.dt;
  int N = (int)std::max(1.0, std::round(P.horizon / dt));

  double v_lo = clamp(robot.v - P.a_v * dt, 0.0, P.v_max);
  double v_hi = clamp(robot.v + P.a_v * dt, 0.0, P.v_max);
  double w_lo = clamp(robot.w - P.a_w * dt, -P.w_max, P.w_max);
  double w_hi = clamp(robot.w + P.a_w * dt, -P.w_max, P.w_max);

  Vec2 to_goal = goal - robot.p;
  double d_goal = norm(to_goal);
  double v_pref = (d_goal > 1.0) ? P.v_max : P.v_max * d_goal;

  auto linspace = [](double a, double b, int n) {
    std::vector<double> xs;
    xs.reserve(n);
    if (n == 1) { xs.push_back(0.5 * (a + b)); return xs; }
    for (int i = 0; i < n; ++i) {
      double t = (double)i / (double)(n - 1);
      xs.push_back(a + (b - a) * t);
    }
    return xs;
  };

  std::vector<double> v_samples = linspace(v_lo, v_hi, 9);
  std::vector<double> w_samples = linspace(w_lo, w_hi, 11);

  std::pair<double, double> best{0.0, 0.0};
  double best_cost = std::numeric_limits<double>::infinity();

  for (double v : v_samples) {
    for (double w : w_samples) {
      Vec2 p = robot.p;
      double th = robot.th;

      double ttc_min = std::numeric_limits<double>::infinity();
      double clear_min = std::numeric_limits<double>::infinity();
      double soc_sum = 0.0;

      for (int k = 0; k < N; ++k) {
        simulate_unicycle(p, th, v, w, dt);
        double t = (k + 1) * dt;

        // predicted humans at time t (constant velocity)
        std::vector<DiscAgent> humans_t = humans;
        for (auto& h : humans_t) { h.p = h.p + h.v * t; }

        for (const auto& h : humans_t) {
          Vec2 rel = Vec2{p.x - h.p.x, p.y - h.p.y};
          double dist = norm(rel) - (robot.r + h.r);
          if (dist < clear_min) clear_min = dist;

          Vec2 v_r = Vec2{v * std::cos(th), v * std::sin(th)};
          Vec2 v_rel = Vec2{v_r.x - h.v.x, v_r.y - h.v.y};
          double ttc = ttc_discs(rel, v_rel, robot.r + h.r);
          if (ttc < ttc_min) ttc_min = ttc;
        }
        soc_sum += social_cost(p, humans_t, P);
      }

      double d_term = norm(goal - p);

      const double eps = 1e-6;
      double c_goal = P.w_goal * d_term;
      double c_clear = P.w_clear * (1.0 / (clear_min + eps));
      double c_ttc = P.w_ttc * (1.0 / (ttc_min + eps));
      double c_soc = P.w_soc * (soc_sum / (double)N);
      double c_vel = P.w_vel * ((v_pref - v) * (v_pref - v));
      double c_turn = P.w_turn * (w * w);
      double c_smooth = P.w_smooth * ((v - robot.v) * (v - robot.v) + 0.1 * (w - robot.w) * (w - robot.w));

      if (clear_min <= 0.0) continue;
      double cost = c_goal + c_clear + c_ttc + c_soc + c_vel + c_turn + c_smooth;

      if (cost < best_cost) {
        best_cost = cost;
        best = {v, w};
      }
    }
  }
  return best;
}

int main() {
  Params P;
  std::vector<DiscAgent> humans = make_crossing_crowd(12, 0.9, 4);

  Robot robot;
  robot.p = Vec2{-8.0, -8.0};
  robot.th = 3.14159265358979323846 / 4.0;
  robot.v = 0.0;
  robot.w = 0.0;
  robot.r = 0.32;

  Vec2 goal{8.0, 8.0};

  const double T_max = 70.0;
  const int steps = (int)(T_max / P.dt);

  std::ofstream csv("robot_traj.csv");
  csv << "t,x,y,th,v,w\n";

  bool collision = false;
  bool reached = false;

  for (int step = 0; step < steps; ++step) {
    double t = step * P.dt;

    auto [v_cmd, w_cmd] = plan_control(robot, goal, humans, P);
    robot.v = v_cmd;
    robot.w = w_cmd;

    simulate_unicycle(robot.p, robot.th, robot.v, robot.w, P.dt);
    update_humans(humans, P.dt, 9.0, 0.01, (std::uint32_t)(4 + step));

    csv << t << "," << robot.p.x << "," << robot.p.y << "," << robot.th << "," << robot.v << "," << robot.w << "\n";

    if (norm(goal - robot.p) < 0.5) { reached = true; break; }

    for (const auto& h : humans) {
      if (norm(robot.p - h.p) <= (robot.r + h.r)) { collision = true; break; }
    }
    if (collision) break;
  }

  csv.close();

  if (collision) {
    std::cout << "Result: collision. See robot_traj.csv\n";
    return 1;
  }
  if (reached) {
    std::cout << "Result: reached goal. See robot_traj.csv\n";
    return 0;
  }
  std::cout << "Result: timeout. See robot_traj.csv\n";
  return 0;
}
