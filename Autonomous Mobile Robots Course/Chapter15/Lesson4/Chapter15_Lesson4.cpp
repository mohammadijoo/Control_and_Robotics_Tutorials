// Chapter15_Lesson4.cpp
// Timed-Elastic-Band (TEB) local planning — minimal educational Gauss-Newton (numeric Jacobian).
//
// Build (example, Linux/macOS with Eigen installed):
//   g++ -O2 -std=c++17 Chapter15_Lesson4.cpp -I /usr/include/eigen3 -o teb_demo
//
// Notes:
// - This is not ROS code; it demonstrates the underlying nonlinear least-squares structure.
// - For production, use analytic Jacobians and a graph optimizer (e.g., g2o / Ceres).

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

struct CircleObstacle {
  double cx, cy, r;
};

static inline double wrap_to_pi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}
static inline double hinge(double x) { return (x > 0.0) ? x : 0.0; }

struct Weights {
  double w_smooth = 10.0;
  double w_align  = 2.0;
  double w_time   = 1.0;
  double w_obst   = 50.0;
  double w_v      = 10.0;
  double w_w      = 5.0;
  double w_a      = 1.0;
  double w_alpha  = 1.0;
};

struct Limits {
  double v_max = 0.8;
  double w_max = 1.2;
  double a_max = 0.8;
  double alpha_max = 1.5;
  double dt_min = 0.05;
  double dt_max = 0.6;
  double d_min  = 0.35;
  double robot_radius = 0.20;
};

struct Pose {
  double x, y, th;
};

static void build_initial(const Pose& start, const Pose& goal, int N,
                          Eigen::VectorXd& xs, Eigen::VectorXd& ys,
                          Eigen::VectorXd& th, Eigen::VectorXd& dts) {
  xs.resize(N); ys.resize(N); th.resize(N); dts.resize(N-1);
  for (int i = 0; i < N; ++i) {
    double a = double(i) / double(N-1);
    xs(i) = (1.0 - a) * start.x + a * goal.x;
    ys(i) = (1.0 - a) * start.y + a * goal.y;
  }
  th(0) = start.th; th(N-1) = goal.th;
  for (int i = 1; i < N-1; ++i) {
    double dx = xs(i+1) - xs(i);
    double dy = ys(i+1) - ys(i);
    th(i) = std::atan2(dy, dx);
  }
  dts.setConstant(0.22);
}

// pack z = [x_mid, y_mid, th_mid, dt]
static Eigen::VectorXd pack_z(const Eigen::VectorXd& xs, const Eigen::VectorXd& ys,
                              const Eigen::VectorXd& th, const Eigen::VectorXd& dts) {
  int N = xs.size();
  int n_mid = N - 2;
  int n_dt  = N - 1;
  Eigen::VectorXd z(3*n_mid + n_dt);
  z.segment(0, n_mid) = xs.segment(1, n_mid);
  z.segment(n_mid, n_mid) = ys.segment(1, n_mid);
  z.segment(2*n_mid, n_mid) = th.segment(1, n_mid);
  z.segment(3*n_mid, n_dt) = dts;
  return z;
}

struct ResidualContext {
  Pose start, goal;
  int N;
  std::vector<CircleObstacle> obstacles;
  Weights w;
  Limits lim;

  void unpack(const Eigen::VectorXd& z, Eigen::VectorXd& xs, Eigen::VectorXd& ys,
              Eigen::VectorXd& th, Eigen::VectorXd& dts) const {
    int n_mid = N - 2;
    int n_dt  = N - 1;
    xs.resize(N); ys.resize(N); th.resize(N); dts.resize(n_dt);

    xs(0) = start.x; ys(0) = start.y; th(0) = start.th;
    xs(N-1) = goal.x; ys(N-1) = goal.y; th(N-1) = goal.th;

    xs.segment(1, n_mid) = z.segment(0, n_mid);
    ys.segment(1, n_mid) = z.segment(n_mid, n_mid);
    th.segment(1, n_mid) = z.segment(2*n_mid, n_mid);
    dts = z.segment(3*n_mid, n_dt);
  }

  Eigen::VectorXd residual(const Eigen::VectorXd& z) const {
    Eigen::VectorXd xs, ys, th, dts;
    unpack(z, xs, ys, th, dts);

    std::vector<double> r;
    r.reserve(500);

    const int n_dt = N - 1;

    // dt objective + bounds
    for (int i = 0; i < n_dt; ++i) {
      r.push_back(std::sqrt(w.w_time) * dts(i));
      r.push_back(std::sqrt(w.w_time) * hinge(lim.dt_min - dts(i)));
      r.push_back(std::sqrt(w.w_time) * hinge(dts(i) - lim.dt_max));
    }

    // smoothness
    for (int i = 1; i < N-1; ++i) {
      double ddx = xs(i+1) - 2.0*xs(i) + xs(i-1);
      double ddy = ys(i+1) - 2.0*ys(i) + ys(i-1);
      r.push_back(std::sqrt(w.w_smooth) * ddx);
      r.push_back(std::sqrt(w.w_smooth) * ddy);
    }

    // align
    for (int i = 0; i < N-1; ++i) {
      double dx = xs(i+1) - xs(i);
      double dy = ys(i+1) - ys(i);
      double ang = std::atan2(dy, dx);
      r.push_back(std::sqrt(w.w_align) * std::sin(wrap_to_pi(th(i) - ang)));
    }

    // obstacle clearance
    for (int i = 0; i < N; ++i) {
      double px = xs(i), py = ys(i);
      double dmin = 1e9;
      for (const auto& o : obstacles) {
        double d = std::hypot(px - o.cx, py - o.cy) - (o.r + lim.robot_radius);
        dmin = std::min(dmin, d);
      }
      r.push_back(std::sqrt(w.w_obst) * hinge(lim.d_min - dmin));
    }

    // v, w bounds and store v,w
    std::vector<double> v(N-1), wv(N-1);
    for (int i = 0; i < N-1; ++i) {
      double dt = std::max(dts(i), 1e-6);
      double dx = xs(i+1) - xs(i);
      double dy = ys(i+1) - ys(i);
      double ds = std::hypot(dx, dy);
      v[i] = ds / dt;
      wv[i] = wrap_to_pi(th(i+1) - th(i)) / dt;

      r.push_back(std::sqrt(w.w_v) * hinge(v[i] - lim.v_max));
      r.push_back(std::sqrt(w.w_w) * hinge(std::abs(wv[i]) - lim.w_max));
    }

    // accel bounds
    for (int i = 0; i < N-2; ++i) {
      double dtm = 0.5 * (dts(i) + dts(i+1));
      dtm = std::max(dtm, 1e-6);
      double a = (v[i+1] - v[i]) / dtm;
      double alpha = (wv[i+1] - wv[i]) / dtm;

      r.push_back(std::sqrt(w.w_a) * hinge(std::abs(a) - lim.a_max));
      r.push_back(std::sqrt(w.w_alpha) * hinge(std::abs(alpha) - lim.alpha_max));
    }

    Eigen::VectorXd out(r.size());
    for (size_t i = 0; i < r.size(); ++i) out(int(i)) = r[i];
    return out;
  }
};

static Eigen::MatrixXd finite_diff_jacobian(const ResidualContext& ctx, const Eigen::VectorXd& z, double eps = 1e-6) {
  Eigen::VectorXd r0 = ctx.residual(z);
  int m = r0.size();
  int n = z.size();
  Eigen::MatrixXd J(m, n);
  for (int j = 0; j < n; ++j) {
    Eigen::VectorXd z1 = z;
    z1(j) += eps;
    Eigen::VectorXd r1 = ctx.residual(z1);
    J.col(j) = (r1 - r0) / eps;
  }
  return J;
}

static Eigen::VectorXd gauss_newton(const ResidualContext& ctx, Eigen::VectorXd z,
                                    int max_iter = 30, double lam = 1e-3) {
  for (int it = 0; it < max_iter; ++it) {
    Eigen::VectorXd r = ctx.residual(z);
    double cost = 0.5 * r.squaredNorm();

    Eigen::MatrixXd J = finite_diff_jacobian(ctx, z);
    Eigen::MatrixXd A = J.transpose() * J + lam * Eigen::MatrixXd::Identity(z.size(), z.size());
    Eigen::VectorXd b = -(J.transpose() * r);

    Eigen::VectorXd dz = A.ldlt().solve(b);

    double step = 1.0;
    for (int ls = 0; ls < 10; ++ls) {
      Eigen::VectorXd z_try = z + step * dz;
      double cost_try = 0.5 * ctx.residual(z_try).squaredNorm();
      if (cost_try < cost) { z = z_try; break; }
      step *= 0.5;
    }

    if ((step * dz).norm() < 1e-5) break;
  }
  return z;
}

int main() {
  Pose start{0.0, 0.0, 0.0};
  Pose goal {4.0, 2.5, 0.0};

  std::vector<CircleObstacle> obstacles = {
    {2.0, 1.2, 0.45},
    {2.8, 2.0, 0.35}
  };

  int N = 18;
  Eigen::VectorXd xs, ys, th, dts;
  build_initial(start, goal, N, xs, ys, th, dts);

  ResidualContext ctx;
  ctx.start = start;
  ctx.goal = goal;
  ctx.N = N;
  ctx.obstacles = obstacles;

  Eigen::VectorXd z0 = pack_z(xs, ys, th, dts);
  Eigen::VectorXd z_opt = gauss_newton(ctx, z0, 35, 1e-3);

  Eigen::VectorXd xs_opt, ys_opt, th_opt, dts_opt;
  ctx.unpack(z_opt, xs_opt, ys_opt, th_opt, dts_opt);

  std::cout << "Optimized first 5 points:\n";
  for (int i = 0; i < std::min(5, N); ++i) {
    std::cout << i << ": (" << xs_opt(i) << ", " << ys_opt(i) << ")\n";
  }
  std::cout << "Total time [s]: " << dts_opt.sum() << "\n";
  return 0;
}
