// Chapter15_Lesson1.cpp
// Pure Pursuit and Geometric Path Tracking (C++17)
// - Core geometry is library-free.
// - Optional ROS2 integration sketch is provided under USE_ROS2.
//
// Build (core demo only):
//   g++ -std=c++17 -O2 Chapter15_Lesson1.cpp -o pp_demo
//
// If you want ROS2:
//   - Define USE_ROS2 and link against rclcpp, nav_msgs, geometry_msgs, etc.
//   - The ROS2 block is a minimal sketch (not a full package).

#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

struct Vec2 {
  double x{0.0}, y{0.0};
};

struct Pose2D {
  double x{0.0}, y{0.0}, theta{0.0};
};

static inline double wrapAngle(double a) {
  while (a <= -M_PI) a += 2.0 * M_PI;
  while (a >   M_PI) a -= 2.0 * M_PI;
  return a;
}

static inline Vec2 operator+(const Vec2& a, const Vec2& b) { return {a.x + b.x, a.y + b.y}; }
static inline Vec2 operator-(const Vec2& a, const Vec2& b) { return {a.x - b.x, a.y - b.y}; }
static inline Vec2 operator*(double s, const Vec2& a) { return {s * a.x, s * a.y}; }

static inline double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
static inline double norm(const Vec2& a) { return std::sqrt(dot(a, a)); }

static inline Vec2 worldToBody(const Pose2D& pose, const Vec2& p_world) {
  // body frame: x forward, y left
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  const Vec2 d{p_world.x - pose.x, p_world.y - pose.y};
  // R^T * d
  return { c * d.x + s * d.y, -s * d.x + c * d.y };
}

static inline Vec2 closestPointOnSegment(const Vec2& p, const Vec2& a, const Vec2& b, double& t_out) {
  const Vec2 ab = b - a;
  const double denom = dot(ab, ab);
  if (denom <= 1e-12) { t_out = 0.0; return a; }
  const double t = dot(p - a, ab) / denom;
  const double tc = std::max(0.0, std::min(1.0, t));
  t_out = tc;
  return a + tc * ab;
}

static inline void polylineArcLength(const std::vector<Vec2>& path, std::vector<double>& s, std::vector<double>& seg) {
  const std::size_t N = path.size();
  s.assign(N, 0.0);
  seg.assign((N >= 2) ? (N - 1) : 0, 0.0);
  for (std::size_t i = 0; i + 1 < N; ++i) {
    seg[i] = norm(path[i + 1] - path[i]);
    s[i + 1] = s[i] + seg[i];
  }
}

static inline Vec2 pointAtArcLength(const std::vector<Vec2>& path,
                                    const std::vector<double>& s,
                                    const std::vector<double>& seg,
                                    double s_query) {
  if (path.empty()) return {0.0, 0.0};
  if (s_query <= 0.0) return path.front();
  if (s_query >= s.back()) return path.back();

  // find j s.t. s[j] <= s_query < s[j+1]
  std::size_t j = 0;
  while (j + 1 < s.size() && s[j + 1] <= s_query) ++j;
  if (j + 1 >= path.size()) return path.back();

  const double ds = s_query - s[j];
  const double t = ds / std::max(seg[j], 1e-12);
  return (1.0 - t) * path[j] + t * path[j + 1];
}

static inline void closestPointOnPolyline(const Vec2& p, const std::vector<Vec2>& path,
                                          Vec2& q_out, std::size_t& i_out, double& t_out, double& d_out) {
  double best_d = std::numeric_limits<double>::infinity();
  Vec2 best_q = path.front();
  std::size_t best_i = 0;
  double best_t = 0.0;

  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    double t = 0.0;
    Vec2 q = closestPointOnSegment(p, path[i], path[i + 1], t);
    const double d = norm(p - q);
    if (d < best_d) {
      best_d = d; best_q = q; best_i = i; best_t = t;
    }
  }
  q_out = best_q; i_out = best_i; t_out = best_t; d_out = best_d;
}

static inline double purePursuitCurvature(const Pose2D& pose, const std::vector<Vec2>& path,
                                         double Ld, Vec2& closest_world, Vec2& look_world) {
  Ld = std::max(Ld, 1e-3);
  const Vec2 p{pose.x, pose.y};

  std::vector<double> s, seg;
  polylineArcLength(path, s, seg);

  std::size_t i = 0;
  double t = 0.0, d = 0.0;
  Vec2 q;
  closestPointOnPolyline(p, path, q, i, t, d);
  closest_world = q;

  const double s_closest = s[i] + t * seg[i];
  look_world = pointAtArcLength(path, s, seg, s_closest + Ld);

  const Vec2 look_b = worldToBody(pose, look_world);
  const double L = std::sqrt(look_b.x * look_b.x + look_b.y * look_b.y);
  if (L <= 1e-9) return 0.0;

  // kappa = 2*y_r / L^2
  return 2.0 * look_b.y / (L * L);
}

static inline Pose2D bicycleStep(const Pose2D& pose, double v, double delta, double wheelbase, double dt) {
  Pose2D nxt = pose;
  nxt.x += v * std::cos(pose.theta) * dt;
  nxt.y += v * std::sin(pose.theta) * dt;
  nxt.theta = wrapAngle(pose.theta + (v / wheelbase) * std::tan(delta) * dt);
  return nxt;
}

int main() {
  // Demo path: simple sinusoid
  std::vector<Vec2> path;
  path.reserve(500);
  for (int i = 0; i < 500; ++i) {
    const double x = 0.05 * i * 25.0; // 0..25
    const double y = 1.8 * std::sin(0.22 * x);
    path.push_back({x, y});
  }

  Pose2D pose{-2.0, -2.0, 0.2};
  const double dt = 0.02;
  const double wheelbase = 0.33;
  const double v = 1.5;

  const double Ld_min = 0.8, Ld_max = 3.5, k_v = 0.8;
  const double kappa_max = 1.6;
  const double delta_max = 32.0 * M_PI / 180.0;

  for (int k = 0; k < static_cast<int>(25.0 / dt); ++k) {
    const double Ld = std::min(Ld_max, std::max(Ld_min, Ld_min + k_v * std::fabs(v)));

    Vec2 q, pL;
    double kappa = purePursuitCurvature(pose, path, Ld, q, pL);
    if (kappa >  kappa_max) kappa =  kappa_max;
    if (kappa < -kappa_max) kappa = -kappa_max;

    double delta = std::atan(wheelbase * kappa);
    if (delta >  delta_max) delta =  delta_max;
    if (delta < -delta_max) delta = -delta_max;

    pose = bicycleStep(pose, v, delta, wheelbase, dt);

    if (k % 50 == 0) {
      std::cout << "k=" << k
                << " pose=(" << pose.x << "," << pose.y << "," << pose.theta << ")"
                << " look=(" << pL.x << "," << pL.y << ")"
                << " kappa=" << kappa << " delta=" << delta
                << "\n";
    }
  }

  std::cout << "Done.\n";
  return 0;
}


/*
==================== OPTIONAL ROS2 SKETCH (NOT BUILT BY DEFAULT) ====================

#define USE_ROS2

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PurePursuitNode : public rclcpp::Node {
public:
  PurePursuitNode() : Node("pure_pursuit_node") {
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
        path_.clear();
        for (const auto& ps : msg->poses) path_.push_back({ps.pose.position.x, ps.pose.position.y});
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { onTimer(); });
  }

  void setPose(const Pose2D& pose) { pose_ = pose; } // supply from TF/odometry in real system

private:
  void onTimer() {
    if (path_.size() < 2) return;
    Vec2 q, pL;
    const double Ld = 1.5;
    double kappa = purePursuitCurvature(pose_, path_, Ld, q, pL);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.7;
    cmd.angular.z = cmd.linear.x * kappa; // unicycle/diff-drive mapping
    cmd_pub_->publish(cmd);
  }

  Pose2D pose_;
  std::vector<Vec2> path_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif

=====================================================================================
*/
