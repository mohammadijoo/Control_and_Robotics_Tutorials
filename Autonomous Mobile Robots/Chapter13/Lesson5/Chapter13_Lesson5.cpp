// Chapter13_Lesson5.cpp
// Minimal ATE evaluation (TUM format) + Umeyama alignment.
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter13_Lesson5.cpp -I /usr/include/eigen3 -o ate_eval
//
// Run:
//   ./ate_eval groundtruth.txt estimated.txt 0.02 1
//
// Args:
//   argv[1]=gt, argv[2]=est, argv[3]=max_dt, argv[4]=allow_scale (0/1)
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_set>
#include <cmath>

struct Traj {
  std::vector<double> t;
  std::vector<Eigen::Vector3d> p;
};

static Traj read_tum(const std::string& path) {
  Traj tr;
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open " + path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    double tt, tx, ty, tz, qx, qy, qz, qw;
    if (!(iss >> tt >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) continue;
    tr.t.push_back(tt);
    tr.p.emplace_back(tx, ty, tz);
  }
  return tr;
}

static std::vector<std::pair<int,int>> associate_by_time(
  const std::vector<double>& ta,
  const std::vector<double>& tb,
  double max_dt)
{
  std::vector<std::pair<int,int>> pairs;
  int j = 0;
  std::unordered_set<int> used;
  for (int i = 0; i < (int)ta.size(); ++i) {
    double t = ta[i];
    while (j + 1 < (int)tb.size() && tb[j] < t) j++;
    std::vector<int> cand;
    for (int jj : {j, j-1}) {
      if (0 <= jj && jj < (int)tb.size() && used.find(jj) == used.end()) cand.push_back(jj);
    }
    if (cand.empty()) continue;
    int best = cand[0];
    for (int jj : cand) if (std::abs(tb[jj] - t) < std::abs(tb[best] - t)) best = jj;
    if (std::abs(tb[best] - t) <= max_dt) {
      pairs.emplace_back(i, best);
      used.insert(best);
    }
  }
  return pairs;
}

static void umeyama(
  const Eigen::MatrixXd& A, // 3xN
  const Eigen::MatrixXd& B, // 3xN
  bool with_scale,
  double& s,
  Eigen::Matrix3d& R,
  Eigen::Vector3d& t)
{
  const int N = (int)A.cols();
  if (N < 3) throw std::runtime_error("Need >=3 points");
  Eigen::Vector3d muA = A.rowwise().mean();
  Eigen::Vector3d muB = B.rowwise().mean();
  Eigen::MatrixXd X = A.colwise() - muA;
  Eigen::MatrixXd Y = B.colwise() - muB;

  Eigen::Matrix3d Sigma = (Y * X.transpose()) / (double)N;

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(Sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Vector3d D = svd.singularValues();

  Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
  if ((U.determinant() * V.determinant()) < 0.0) S(2,2) = -1.0;

  R = U * S * V.transpose();

  if (with_scale) {
    double varA = (X.array() * X.array()).sum() / (double)N;
    s = (D.transpose() * S.diagonal()) / (varA + 1e-12);
  } else {
    s = 1.0;
  }

  t = muB - s * (R * muA);
}

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "Usage: " << argv[0] << " gt.txt est.txt max_dt allow_scale(0/1)\n";
    return 2;
  }
  const std::string gt_path = argv[1];
  const std::string est_path = argv[2];
  const double max_dt = std::stod(argv[3]);
  const bool allow_scale = (std::stoi(argv[4]) != 0);

  Traj gt = read_tum(gt_path);
  Traj est = read_tum(est_path);

  auto pairs = associate_by_time(est.t, gt.t, max_dt);
  if ((int)pairs.size() < 3) {
    std::cerr << "Too few associated poses: " << pairs.size() << "\n";
    return 3;
  }

  const int N = (int)pairs.size();
  Eigen::MatrixXd A(3, N), B(3, N);
  for (int k = 0; k < N; ++k) {
    A.col(k) = est.p[pairs[k].first];
    B.col(k) = gt.p[pairs[k].second];
  }

  double s;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  umeyama(A, B, allow_scale, s, R, t);

  Eigen::MatrixXd A_al = (s * (R * A)).colwise() + t;
  Eigen::MatrixXd E = A_al - B;
  Eigen::VectorXd se = E.colwise().squaredNorm();

  double rmse = std::sqrt(se.mean());
  double med;
  {
    std::vector<double> e;
    e.reserve(N);
    for (int k = 0; k < N; ++k) e.push_back(std::sqrt(se(k)));
    std::nth_element(e.begin(), e.begin() + e.size()/2, e.end());
    med = e[e.size()/2];
  }
  double mx = std::sqrt(se.maxCoeff());

  std::cout << "pairs " << N << "\n";
  std::cout << "scale " << s << "\n";
  std::cout << "rmse_m " << rmse << "\n";
  std::cout << "median_m " << med << "\n";
  std::cout << "max_m " << mx << "\n";
  return 0;
}
