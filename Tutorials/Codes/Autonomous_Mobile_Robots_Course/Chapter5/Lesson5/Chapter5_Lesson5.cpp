/*
Chapter5_Lesson5.cpp

Lab: Characterizing Odometry Error (Differential Drive)

This C++ program:
1) Loads a CSV log with: t,nL,nR,x_odom,y_odom,th_odom,x_gt,y_gt,th_gt,(optional)trial_id.
2) Computes endpoint error, RMSE, and drift-per-meter per trial using "logged odom".
3) Calibrates (rL, rR, b) by Gauss-Newton on increment residuals (using ground truth).
4) Reconstructs odometry from encoders using calibrated params and re-evaluates metrics.

Dependencies:
- C++17

Build:
  g++ -O2 -std=c++17 Chapter5_Lesson5.cpp -o ch5_l5

Run:
  ./ch5_l5 --csv your_log.csv --ticks_per_rev 4096 --r_nom 0.05 --b_nom 0.30 --w_theta 1.0
*/

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#include <limits>

struct Params {
  double rL{0.05};
  double rR{0.05};
  double b{0.30};
};

static inline double wrap_angle(double th) {
  double a = std::fmod(th + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

struct Row {
  double t{0};
  double nL{0};
  double nR{0};
  double x_odom{0}, y_odom{0}, th_odom{0};
  double x_gt{0}, y_gt{0}, th_gt{0};
  int trial_id{0};
};

static bool parse_csv_line(const std::string& line, std::vector<std::string>& out) {
  out.clear();
  std::string token;
  std::stringstream ss(line);
  while (std::getline(ss, token, ',')) {
    out.push_back(token);
  }
  return !out.empty();
}

static std::vector<Row> load_csv(const std::string& path) {
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("Cannot open CSV: " + path);
  }

  std::string header;
  std::getline(f, header);
  std::vector<std::string> cols;
  parse_csv_line(header, cols);

  auto col_index = [&](const std::string& name) -> int {
    for (int i = 0; i < (int)cols.size(); ++i) {
      if (cols[i] == name) return i;
    }
    return -1;
  };

  const int it = col_index("t");
  const int inL = col_index("nL");
  const int inR = col_index("nR");
  const int ixO = col_index("x_odom");
  const int iyO = col_index("y_odom");
  const int ithO = col_index("th_odom");
  const int ixG = col_index("x_gt");
  const int iyG = col_index("y_gt");
  const int ithG = col_index("th_gt");
  const int itr = col_index("trial_id");

  const std::vector<std::string> req = {"t","nL","nR","x_odom","y_odom","th_odom","x_gt","y_gt","th_gt"};
  for (const auto& r : req) {
    if (col_index(r) < 0) {
      throw std::runtime_error("Missing required column: " + r);
    }
  }

  std::vector<Row> rows;
  std::string line;
  std::vector<std::string> toks;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    parse_csv_line(line, toks);
    auto getd = [&](int idx) -> double {
      if (idx < 0 || idx >= (int)toks.size()) return 0.0;
      return std::stod(toks[idx]);
    };
    Row r;
    r.t = getd(it);
    r.nL = getd(inL);
    r.nR = getd(inR);
    r.x_odom = getd(ixO);
    r.y_odom = getd(iyO);
    r.th_odom = wrap_angle(getd(ithO));
    r.x_gt = getd(ixG);
    r.y_gt = getd(iyG);
    r.th_gt = wrap_angle(getd(ithG));
    r.trial_id = (itr >= 0 && itr < (int)toks.size()) ? (int)std::round(getd(itr)) : 0;
    rows.push_back(r);
  }
  return rows;
}

struct Metrics {
  int trial_id{0};
  double rmse_pos{0};
  double rmse_th{0};
  double end_pos{0};
  double end_th{0};
  double s_total{0};
  double drift_per_m{0};
};

static Metrics compute_metrics_logged(const std::vector<Row>& v, int trial_id) {
  const int N = (int)v.size();
  std::vector<double> epos(N), eth(N);
  for (int k = 0; k < N; ++k) {
    const double ex = v[k].x_odom - v[k].x_gt;
    const double ey = v[k].y_odom - v[k].y_gt;
    const double e_th = wrap_angle(v[k].th_odom - v[k].th_gt);
    epos[k] = std::sqrt(ex*ex + ey*ey);
    eth[k] = e_th;
  }
  auto mean_sq = [&](const std::vector<double>& a) -> double {
    double s = 0;
    for (double x : a) s += x*x;
    return s / std::max(1, (int)a.size());
  };
  Metrics m;
  m.trial_id = trial_id;
  m.rmse_pos = std::sqrt(mean_sq(epos));
  m.rmse_th  = std::sqrt(mean_sq(eth));
  // endpoint
  const double exN = v.back().x_odom - v.back().x_gt;
  const double eyN = v.back().y_odom - v.back().y_gt;
  m.end_pos = std::sqrt(exN*exN + eyN*eyN);
  m.end_th  = std::abs(wrap_angle(v.back().th_odom - v.back().th_gt));
  // gt path length
  double s_total = 0;
  for (int k = 1; k < N; ++k) {
    const double dx = v[k].x_gt - v[k-1].x_gt;
    const double dy = v[k].y_gt - v[k-1].y_gt;
    s_total += std::sqrt(dx*dx + dy*dy);
  }
  m.s_total = s_total;
  m.drift_per_m = (s_total > 1e-12) ? (m.end_pos / s_total) : std::numeric_limits<double>::quiet_NaN();
  return m;
}

static inline double ticks_to_dphi(double dn, double ticks_per_rev) {
  return (2.0 * M_PI / ticks_per_rev) * dn;
}

static void build_increments(
  const std::vector<Row>& v,
  double ticks_per_rev,
  std::vector<double>& dphiL,
  std::vector<double>& dphiR,
  std::vector<double>& th_prev,
  std::vector<double>& dx_gt,
  std::vector<double>& dy_gt,
  std::vector<double>& dth_gt
) {
  const int N = (int)v.size();
  dphiL.assign(N, 0.0);
  dphiR.assign(N, 0.0);
  th_prev.assign(N, 0.0);
  dx_gt.assign(N, 0.0);
  dy_gt.assign(N, 0.0);
  dth_gt.assign(N, 0.0);

  for (int k = 1; k < N; ++k) {
    const double dnL = v[k].nL - v[k-1].nL;
    const double dnR = v[k].nR - v[k-1].nR;
    dphiL[k] = ticks_to_dphi(dnL, ticks_per_rev);
    dphiR[k] = ticks_to_dphi(dnR, ticks_per_rev);

    dx_gt[k] = v[k].x_gt - v[k-1].x_gt;
    dy_gt[k] = v[k].y_gt - v[k-1].y_gt;
    dth_gt[k] = wrap_angle(v[k].th_gt - v[k-1].th_gt);

    th_prev[k] = v[k-1].th_gt;
  }
  th_prev[0] = v[0].th_gt;
}

static void stack_residuals_and_jacobian(
  const std::vector<Row>& v,
  double ticks_per_rev,
  const Params& p,
  double w_theta,
  std::vector<double>& r,
  std::vector<double>& J  // row-major (3N x 3)
) {
  std::vector<double> dphiL, dphiR, th_prev, dx_gt, dy_gt, dth_gt;
  build_increments(v, ticks_per_rev, dphiL, dphiR, th_prev, dx_gt, dy_gt, dth_gt);

  const int Nfull = (int)v.size();
  const int N = Nfull - 1; // skip k=0
  r.assign(3*N, 0.0);
  J.assign(3*N*3, 0.0);

  for (int k = 1; k < Nfull; ++k) {
    const double sL = p.rL * dphiL[k];
    const double sR = p.rR * dphiR[k];
    const double ds = 0.5 * (sR + sL);
    const double dth = (sR - sL) / p.b;

    const double a = wrap_angle(th_prev[k] + 0.5 * dth);
    const double ca = std::cos(a);
    const double sa = std::sin(a);

    const double dx = ds * ca;
    const double dy = ds * sa;

    const double rx = dx - dx_gt[k];
    const double ry = dy - dy_gt[k];
    const double rth = wrap_angle(dth - dth_gt[k]);

    const int i = k - 1;
    r[3*i + 0] = rx;
    r[3*i + 1] = ry;
    r[3*i + 2] = w_theta * rth;

    // Derivatives
    const double dds_drL = 0.5 * dphiL[k];
    const double dds_drR = 0.5 * dphiR[k];

    const double ddth_drL = -(dphiL[k]) / p.b;
    const double ddth_drR = (dphiR[k]) / p.b;
    const double ddth_db  = -(sR - sL) / (p.b * p.b);

    const double ddx_drL = ca * dds_drL + ds * (-sa) * (0.5 * ddth_drL);
    const double ddx_drR = ca * dds_drR + ds * (-sa) * (0.5 * ddth_drR);
    const double ddx_db  = ds * (-sa) * (0.5 * ddth_db);

    const double ddy_drL = sa * dds_drL + ds * ca * (0.5 * ddth_drL);
    const double ddy_drR = sa * dds_drR + ds * ca * (0.5 * ddth_drR);
    const double ddy_db  = ds * ca * (0.5 * ddth_db);

    const double ddth_drL_s = w_theta * ddth_drL;
    const double ddth_drR_s = w_theta * ddth_drR;
    const double ddth_db_s  = w_theta * ddth_db;

    // Fill row-major J (3N x 3)
    // row 3*i + 0
    J[(3*i + 0)*3 + 0] = ddx_drL;
    J[(3*i + 0)*3 + 1] = ddx_drR;
    J[(3*i + 0)*3 + 2] = ddx_db;
    // row 3*i + 1
    J[(3*i + 1)*3 + 0] = ddy_drL;
    J[(3*i + 1)*3 + 1] = ddy_drR;
    J[(3*i + 1)*3 + 2] = ddy_db;
    // row 3*i + 2
    J[(3*i + 2)*3 + 0] = ddth_drL_s;
    J[(3*i + 2)*3 + 1] = ddth_drR_s;
    J[(3*i + 2)*3 + 2] = ddth_db_s;
  }
}

static bool solve_3x3(const double A[9], const double b[3], double x[3]) {
  // Gaussian elimination for 3x3
  double M[3][4] = {
    {A[0], A[1], A[2], b[0]},
    {A[3], A[4], A[5], b[1]},
    {A[6], A[7], A[8], b[2]}
  };
  for (int i = 0; i < 3; ++i) {
    // pivot
    int piv = i;
    for (int r = i+1; r < 3; ++r) {
      if (std::fabs(M[r][i]) > std::fabs(M[piv][i])) piv = r;
    }
    if (std::fabs(M[piv][i]) < 1e-18) return false;
    if (piv != i) {
      for (int c = i; c < 4; ++c) std::swap(M[i][c], M[piv][c]);
    }
    // normalize
    const double inv = 1.0 / M[i][i];
    for (int c = i; c < 4; ++c) M[i][c] *= inv;
    // eliminate
    for (int r = 0; r < 3; ++r) {
      if (r == i) continue;
      const double f = M[r][i];
      for (int c = i; c < 4; ++c) M[r][c] -= f * M[i][c];
    }
  }
  x[0] = M[0][3];
  x[1] = M[1][3];
  x[2] = M[2][3];
  return true;
}

static Params gauss_newton_calibrate(
  const std::vector<Row>& v,
  double ticks_per_rev,
  Params p,
  double w_theta,
  int max_iter = 15,
  double damping = 1e-9
) {
  for (int it = 0; it < max_iter; ++it) {
    std::vector<double> r, J;
    stack_residuals_and_jacobian(v, ticks_per_rev, p, w_theta, r, J);
    const int m = (int)r.size(); // 3N

    // Compute A = J^T J, g = J^T r
    double A[9] = {0};
    double g[3] = {0};

    for (int row = 0; row < m; ++row) {
      const double* Jr = &J[row*3];
      for (int a = 0; a < 3; ++a) {
        g[a] += Jr[a] * r[row];
        for (int b = 0; b < 3; ++b) {
          A[a*3 + b] += Jr[a] * Jr[b];
        }
      }
    }
    // damping
    A[0] += damping; A[4] += damping; A[8] += damping;

    // Solve A dp = -g
    double bvec[3] = {-g[0], -g[1], -g[2]};
    double dp[3] = {0};
    if (!solve_3x3(A, bvec, dp)) break;

    const double norm_dp = std::sqrt(dp[0]*dp[0] + dp[1]*dp[1] + dp[2]*dp[2]);
    if (norm_dp < 1e-12) break;

    // Simple positivity line search
    double alpha = 1.0;
    for (int ls = 0; ls < 10; ++ls) {
      Params cand{p.rL + alpha*dp[0], p.rR + alpha*dp[1], p.b + alpha*dp[2]};
      if (cand.rL > 0 && cand.rR > 0 && cand.b > 0) {
        p = cand;
        break;
      }
      alpha *= 0.5;
    }
  }
  return p;
}

static Metrics compute_metrics_reconstructed(
  const std::vector<Row>& v,
  int trial_id,
  double ticks_per_rev,
  const Params& p
) {
  const int N = (int)v.size();
  std::vector<double> dphiL(N, 0.0), dphiR(N, 0.0);

  for (int k = 1; k < N; ++k) {
    const double dnL = v[k].nL - v[k-1].nL;
    const double dnR = v[k].nR - v[k-1].nR;
    dphiL[k] = ticks_to_dphi(dnL, ticks_per_rev);
    dphiR[k] = ticks_to_dphi(dnR, ticks_per_rev);
  }

  // Integrate odom from gt start pose for alignment
  double x = v[0].x_gt;
  double y = v[0].y_gt;
  double th = v[0].th_gt;

  std::vector<double> x_hat(N), y_hat(N), th_hat(N);
  x_hat[0] = x; y_hat[0] = y; th_hat[0] = th;

  for (int k = 1; k < N; ++k) {
    const double sL = p.rL * dphiL[k];
    const double sR = p.rR * dphiR[k];
    const double ds = 0.5 * (sR + sL);
    const double dth = (sR - sL) / p.b;
    const double th_mid = wrap_angle(th + 0.5 * dth);

    x += ds * std::cos(th_mid);
    y += ds * std::sin(th_mid);
    th = wrap_angle(th + dth);

    x_hat[k] = x; y_hat[k] = y; th_hat[k] = th;
  }

  // Compute errors vs gt
  std::vector<double> epos(N), eth(N);
  for (int k = 0; k < N; ++k) {
    const double ex = x_hat[k] - v[k].x_gt;
    const double ey = y_hat[k] - v[k].y_gt;
    epos[k] = std::sqrt(ex*ex + ey*ey);
    eth[k] = wrap_angle(th_hat[k] - v[k].th_gt);
  }

  auto mean_sq = [&](const std::vector<double>& a) -> double {
    double s = 0;
    for (double x : a) s += x*x;
    return s / std::max(1, (int)a.size());
  };

  Metrics m;
  m.trial_id = trial_id;
  m.rmse_pos = std::sqrt(mean_sq(epos));
  m.rmse_th  = std::sqrt(mean_sq(eth));

  // endpoint
  const double exN = x_hat.back() - v.back().x_gt;
  const double eyN = y_hat.back() - v.back().y_gt;
  m.end_pos = std::sqrt(exN*exN + eyN*eyN);
  m.end_th  = std::abs(wrap_angle(th_hat.back() - v.back().th_gt));

  // gt path length
  double s_total = 0;
  for (int k = 1; k < N; ++k) {
    const double dx = v[k].x_gt - v[k-1].x_gt;
    const double dy = v[k].y_gt - v[k-1].y_gt;
    s_total += std::sqrt(dx*dx + dy*dy);
  }
  m.s_total = s_total;
  m.drift_per_m = (s_total > 1e-12) ? (m.end_pos / s_total) : std::numeric_limits<double>::quiet_NaN();
  return m;
}

static void print_metrics_table(const std::vector<Metrics>& ms) {
  std::cout << "trial_id, rmse_pos, rmse_th, end_pos, end_th, s_total, drift_per_m\n";
  for (const auto& m : ms) {
    std::cout << m.trial_id << ", "
              << m.rmse_pos << ", "
              << m.rmse_th << ", "
              << m.end_pos << ", "
              << m.end_th << ", "
              << m.s_total << ", "
              << m.drift_per_m << "\n";
  }
}

int main(int argc, char** argv) {
  std::string csv_path;
  double ticks_per_rev = 0.0;
  double r_nom = 0.05;
  double b_nom = 0.30;
  double w_theta = 1.0;

  // simple arg parsing
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](double& x) {
      if (i+1 >= argc) throw std::runtime_error("Missing value after " + a);
      x = std::stod(argv[++i]);
    };
    auto nexts = [&](std::string& s) {
      if (i+1 >= argc) throw std::runtime_error("Missing value after " + a);
      s = argv[++i];
    };

    if (a == "--csv") nexts(csv_path);
    else if (a == "--ticks_per_rev") next(ticks_per_rev);
    else if (a == "--r_nom") next(r_nom);
    else if (a == "--b_nom") next(b_nom);
    else if (a == "--w_theta") next(w_theta);
  }

  if (csv_path.empty() || ticks_per_rev <= 0.0) {
    std::cerr << "Usage: ./ch5_l5 --csv log.csv --ticks_per_rev 4096 [--r_nom 0.05 --b_nom 0.30 --w_theta 1.0]\n";
    return 1;
  }

  std::vector<Row> rows = load_csv(csv_path);

  // Group by trial
  std::map<int, std::vector<Row>> trials;
  for (const auto& r : rows) {
    trials[r.trial_id].push_back(r);
  }

  // Metrics from logged odom
  std::vector<Metrics> m_logged;
  for (auto& kv : trials) {
    m_logged.push_back(compute_metrics_logged(kv.second, kv.first));
  }

  std::cout << "=== Metrics (logged odom) ===\n";
  print_metrics_table(m_logged);
  std::cout << "\n";

  // Calibrate on concatenated data
  Params p0{r_nom, r_nom, b_nom};
  Params p_hat = gauss_newton_calibrate(rows, ticks_per_rev, p0, w_theta);

  std::cout << "=== Calibration Result ===\n";
  std::cout << "rL_hat = " << p_hat.rL << " m\n";
  std::cout << "rR_hat = " << p_hat.rR << " m\n";
  std::cout << "b_hat  = " << p_hat.b  << " m\n\n";

  // Metrics using reconstructed odom with calibrated parameters
  std::vector<Metrics> m_cal;
  for (auto& kv : trials) {
    m_cal.push_back(compute_metrics_reconstructed(kv.second, kv.first, ticks_per_rev, p_hat));
  }

  std::cout << "=== Metrics (reconstructed, calibrated) ===\n";
  print_metrics_table(m_cal);
  std::cout << "\n";

  return 0;
}
