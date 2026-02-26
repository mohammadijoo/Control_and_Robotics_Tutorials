// Chapter16_Lesson4.cpp
// Prediction-Aware Local Navigation (sampling MPC with chance-constraint penalty)
// Minimal, self-contained (no Eigen). Compile: g++ -O2 -std=c++17 Chapter16_Lesson4.cpp -o demo

#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

static double wrap_angle(double a) {
  const double pi = 3.14159265358979323846;
  a = std::fmod(a + pi, 2.0 * pi);
  if (a < 0.0) a += 2.0 * pi;
  return a - pi;
}

struct Vec2 {
  double x{0.0}, y{0.0};
};

struct Vec3 {
  double x{0.0}, y{0.0}, th{0.0};
};

struct Mat2 {
  // row-major
  double a11{0.0}, a12{0.0}, a21{0.0}, a22{0.0};
};

struct Track {
  // mean: [px, py, vx, vy]
  double px{0.0}, py{0.0}, vx{0.0}, vy{0.0};
  // covariance for position block (2x2) only (for penalty)
  Mat2 SigmaPos;
  double radius{0.35};
  std::string name{"obs"};

  void predict(double dt, double q_pos = 0.05, double q_vel = 0.2) {
    // constant velocity mean update
    px += dt * vx;
    py += dt * vy;

    // crude covariance inflation (position grows with velocity uncertainty)
    // For teaching: SigmaPos <- SigmaPos + dt^2 * q_vel^2 I + q_pos^2 I
    const double add = (dt * q_vel) * (dt * q_vel) + q_pos * q_pos;
    SigmaPos.a11 += add;
    SigmaPos.a22 += add;
  }
};

static Vec3 unicycle_step(const Vec3& x, double v, double w, double dt) {
  Vec3 nx;
  nx.x = x.x + dt * v * std::cos(x.th);
  nx.y = x.y + dt * v * std::sin(x.th);
  nx.th = wrap_angle(x.th + dt * w);
  return nx;
}

static double z_value(double delta) {
  // table for z_{1-delta}
  if (std::abs(delta - 0.1) < 1e-12) return 1.281551565545;
  if (std::abs(delta - 0.05) < 1e-12) return 1.644853626951;
  if (std::abs(delta - 0.02) < 1e-12) return 2.053748910631;
  if (std::abs(delta - 0.01) < 1e-12) return 2.326347874041;
  if (std::abs(delta - 0.005) < 1e-12) return 2.575829303549;
  if (std::abs(delta - 0.001) < 1e-12) return 3.090232306168;
  // fallback
  return 2.326347874041;
}

static double chance_penalty_point(const Vec2& p, const Track& t, double R_safe, double delta) {
  // mu_r = p - mu_o
  const double mux = p.x - t.px;
  const double muy = p.y - t.py;
  const double dist = std::sqrt(mux * mux + muy * muy) + 1e-9;

  // n = mu / ||mu||
  const double nx = mux / dist;
  const double ny = muy / dist;

  // s^2 = n^T Sigma n
  const double s2 = nx * (t.SigmaPos.a11 * nx + t.SigmaPos.a12 * ny)
                  + ny * (t.SigmaPos.a21 * nx + t.SigmaPos.a22 * ny);
  const double s = std::sqrt(std::max(0.0, s2) + 1e-12);

  const double z = z_value(delta);
  const double margin = z * s;
  const double violation = std::max(0.0, margin - (dist - R_safe));
  return violation * violation;
}

static double rollout_cost(
    const Vec3& x0,
    double v,
    double w,
    const Vec2& goal,
    const std::vector<Track>& tracks0,
    double dt,
    int N,
    double delta,
    double robot_radius) {

  const double w_goal = 1.0;
  const double w_ctrl = 0.05;
  const double w_risk = 10.0;

  Vec3 x = x0;
  std::vector<Track> tracks = tracks0; // copy to predict in-place

  double cost = 0.0;
  for (int k = 0; k < N; ++k) {
    for (auto& t : tracks) t.predict(dt);

    x = unicycle_step(x, v, w, dt);

    const double ex = x.x - goal.x;
    const double ey = x.y - goal.y;
    cost += w_goal * (ex * ex + ey * ey);

    cost += w_ctrl * (v * v + w * w);

    Vec2 p{x.x, x.y};
    double risk_k = 0.0;
    for (const auto& t : tracks) {
      const double R_safe = robot_radius + t.radius;
      risk_k += chance_penalty_point(p, t, R_safe, delta);
    }
    cost += w_risk * risk_k;
  }
  return cost;
}

int main() {
  Vec3 x0{0.0, 0.0, 0.0};
  Vec2 goal{6.0, 0.0};

  std::vector<Track> tracks;
  Track p1;
  p1.px = 3.0; p1.py = 1.0; p1.vx = 0.0; p1.vy = -0.6; p1.radius = 0.35; p1.name = "p1";
  p1.SigmaPos = Mat2{0.15 * 0.15, 0.0, 0.0, 0.15 * 0.15};
  tracks.push_back(p1);

  Track p2;
  p2.px = 4.0; p2.py = -1.2; p2.vx = 0.0; p2.vy = 0.7; p2.radius = 0.35; p2.name = "p2";
  p2.SigmaPos = Mat2{0.15 * 0.15, 0.0, 0.0, 0.15 * 0.15};
  tracks.push_back(p2);

  const double dt = 0.1;
  const int N = 25;
  const double delta = 0.01;
  const double robot_radius = 0.25;

  const double vmin = 0.0, vmax = 1.2;
  const double wmin = -1.8, wmax = 1.8;
  const int n_v = 13;
  const int n_w = 31;

  double best_v = 0.0, best_w = 0.0;
  double best_J = std::numeric_limits<double>::infinity();

  for (int i = 0; i < n_v; ++i) {
    const double v = vmin + (vmax - vmin) * (static_cast<double>(i) / (n_v - 1));
    for (int j = 0; j < n_w; ++j) {
      const double w = wmin + (wmax - wmin) * (static_cast<double>(j) / (n_w - 1));
      const double J = rollout_cost(x0, v, w, goal, tracks, dt, N, delta, robot_radius);
      if (J < best_J) {
        best_J = J;
        best_v = v;
        best_w = w;
      }
    }
  }

  std::cout << "Best control u* = [v,w] = [" << best_v << ", " << best_w << "], cost = " << best_J << "\n";
  return 0;
}
