// Chapter19_Lesson3.cpp
// Standard Datasets and Simulated Benchmarks for AMR
// C++17 example (Eigen) for SE(2) trajectory benchmark metrics.
//
// Build example:
//   g++ -std=c++17 Chapter19_Lesson3.cpp -O2 -I /path/to/eigen -o ch19_l3
//
// CSV format expected: t,x,y,yaw

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

struct TrajectorySE2 {
  std::vector<double> t, x, y, yaw;
};

static double WrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

TrajectorySE2 LoadCSV(const std::string& path) {
  std::ifstream fin(path);
  if (!fin) throw std::runtime_error("Cannot open file: " + path);

  TrajectorySE2 tr;
  std::string line;
  std::getline(fin, line);  // header
  while (std::getline(fin, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> row;
    while (std::getline(ss, cell, ',')) row.push_back(std::stod(cell));
    if (row.size() != 4) throw std::runtime_error("CSV row must have 4 columns");
    tr.t.push_back(row[0]);
    tr.x.push_back(row[1]);
    tr.y.push_back(row[2]);
    tr.yaw.push_back(row[3]);
  }
  return tr;
}

size_t FindSegment(const std::vector<double>& t, double tq) {
  if (tq < t.front() || tq > t.back()) throw std::runtime_error("Query time out of bounds");
  auto it = std::lower_bound(t.begin(), t.end(), tq);
  if (it == t.begin()) return 0;
  if (it == t.end()) return t.size() - 2;
  size_t j = static_cast<size_t>(it - t.begin());
  if (*it == tq) return (j == t.size() - 1) ? j - 1 : j;
  return j - 1;
}

Eigen::Vector3d InterpPose(const TrajectorySE2& tr, double tq) {
  size_t i = FindSegment(tr.t, tq);
  size_t j = i + 1;
  double a = (tq - tr.t[i]) / (tr.t[j] - tr.t[i]);

  double x = (1.0 - a) * tr.x[i] + a * tr.x[j];
  double y = (1.0 - a) * tr.y[i] + a * tr.y[j];
  double c = (1.0 - a) * std::cos(tr.yaw[i]) + a * std::cos(tr.yaw[j]);
  double s = (1.0 - a) * std::sin(tr.yaw[i]) + a * std::sin(tr.yaw[j]);
  double yaw = std::atan2(s, c);
  return {x, y, yaw};
}

std::pair<Eigen::Matrix2d, Eigen::Vector2d> Align2D(
    const std::vector<Eigen::Vector2d>& est,
    const std::vector<Eigen::Vector2d>& gt) {
  if (est.size() != gt.size() || est.empty()) throw std::runtime_error("Invalid alignment input");

  Eigen::Vector2d mu_e = Eigen::Vector2d::Zero();
  Eigen::Vector2d mu_g = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < est.size(); ++i) {
    mu_e += est[i];
    mu_g += gt[i];
  }
  mu_e /= static_cast<double>(est.size());
  mu_g /= static_cast<double>(gt.size());

  Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
  for (size_t i = 0; i < est.size(); ++i) {
    H += (est[i] - mu_e) * (gt[i] - mu_g).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0.0) {
    Eigen::Matrix2d V = svd.matrixV();
    V.col(1) *= -1.0;
    R = V * svd.matrixU().transpose();
  }
  Eigen::Vector2d t = mu_g - R * mu_e;
  return {R, t};
}

struct Metrics {
  double ate_rmse_m = 0.0;
  double rpe_rmse_m = 0.0;
  double rpe_yaw_mean_rad = 0.0;
  size_t pairs = 0;
};

Eigen::Vector3d RelativeSE2(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) {
  double dxw = p1.x() - p0.x();
  double dyw = p1.y() - p0.y();
  double c = std::cos(p0.z()), s = std::sin(p0.z());
  double dx = c * dxw + s * dyw;
  double dy = -s * dxw + c * dyw;
  double dth = WrapAngle(p1.z() - p0.z());
  return {dx, dy, dth};
}

Metrics Evaluate(const TrajectorySE2& gt, const TrajectorySE2& est, double delta_t) {
  std::vector<Eigen::Vector3d> gt_i, est_p;
  std::vector<Eigen::Vector2d> gt_xy, est_xy;
  gt_i.reserve(est.t.size());
  est_p.reserve(est.t.size());
  for (size_t i = 0; i < est.t.size(); ++i) {
    Eigen::Vector3d g = InterpPose(gt, est.t[i]);
    Eigen::Vector3d e(est.x[i], est.y[i], est.yaw[i]);
    gt_i.push_back(g);
    est_p.push_back(e);
    gt_xy.push_back(g.head<2>());
    est_xy.push_back(e.head<2>());
  }

  auto [R, t] = Align2D(est_xy, gt_xy);
  std::vector<Eigen::Vector3d> est_aligned = est_p;
  for (auto& p : est_aligned) p.head<2>() = R * p.head<2>() + t;

  Metrics m;
  double sse = 0.0;
  for (size_t i = 0; i < est_aligned.size(); ++i) {
    Eigen::Vector2d e = est_aligned[i].head<2>() - gt_i[i].head<2>();
    sse += e.squaredNorm();
  }
  m.ate_rmse_m = std::sqrt(sse / static_cast<double>(est_aligned.size()));

  std::vector<double> rpe_trans, rpe_yaw;
  for (size_t i = 0; i < est.t.size(); ++i) {
    double target = est.t[i] + delta_t;
    auto it = std::lower_bound(est.t.begin(), est.t.end(), target);
    if (it == est.t.end()) continue;
    size_t j = static_cast<size_t>(it - est.t.begin());

    Eigen::Vector3d d_gt = RelativeSE2(gt_i[i], gt_i[j]);
    Eigen::Vector3d d_est = RelativeSE2(est_aligned[i], est_aligned[j]);
    Eigen::Vector3d diff = d_est - d_gt;
    diff.z() = WrapAngle(diff.z());

    rpe_trans.push_back(diff.head<2>().norm());
    rpe_yaw.push_back(std::abs(diff.z()));
  }

  m.pairs = rpe_trans.size();
  if (!rpe_trans.empty()) {
    double sse_r = 0.0, sum_y = 0.0;
    for (double v : rpe_trans) sse_r += v * v;
    for (double v : rpe_yaw) sum_y += v;
    m.rpe_rmse_m = std::sqrt(sse_r / static_cast<double>(rpe_trans.size()));
    m.rpe_yaw_mean_rad = sum_y / static_cast<double>(rpe_yaw.size());
  }
  return m;
}

int main(int argc, char** argv) {
  try {
    if (argc < 3) {
      std::cerr << "Usage: " << argv[0] << " gt.csv est.csv [delta_t]\n";
      return 1;
    }
    double delta_t = (argc >= 4) ? std::stod(argv[3]) : 1.0;
    TrajectorySE2 gt = LoadCSV(argv[1]);
    TrajectorySE2 est = LoadCSV(argv[2]);
    Metrics m = Evaluate(gt, est, delta_t);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "ATE_trans_rmse_m=" << m.ate_rmse_m << "\n";
    std::cout << "RPE_trans_rmse_m=" << m.rpe_rmse_m << "\n";
    std::cout << "RPE_yaw_mean_rad=" << m.rpe_yaw_mean_rad << "\n";
    std::cout << "Pairs=" << m.pairs << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 2;
  }
}
