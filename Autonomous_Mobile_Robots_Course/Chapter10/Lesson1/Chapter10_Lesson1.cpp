// Chapter10_Lesson1.cpp
// Point-Cloud / Scan Registration Goals — minimal educational C++ example
//
// Closed-form 2D point-set registration (paired correspondences) via SVD (Kabsch/Procrustes).
// Dependencies: Eigen3.
//
// Build example (Linux/macOS):
//   g++ -O2 -std=c++17 Chapter10_Lesson1.cpp -I /usr/include/eigen3 -o Chapter10_Lesson1
//
// Windows (MSVC): add Eigen include directory to project include paths.

#include <iostream>
#include <random>
#include <vector>
#include <Eigen/Dense>

using Mat2 = Eigen::Matrix2d;
using Vec2 = Eigen::Vector2d;

struct Pose2 {
  Mat2 R;
  Vec2 t;
};

Mat2 Rot2(double theta) {
  double c = std::cos(theta), s = std::sin(theta);
  Mat2 R;
  R << c, -s,
       s,  c;
  return R;
}

Pose2 Kabsch2D(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q) {
  // P, Q: (N,2)
  if (P.rows() != Q.rows() || P.cols() != 2 || Q.cols() != 2) {
    throw std::runtime_error("P and Q must be (N,2) and same size.");
  }

  Vec2 pbar = P.colwise().mean();
  Vec2 qbar = Q.colwise().mean();

  Eigen::MatrixXd X = P.rowwise() - pbar.transpose();
  Eigen::MatrixXd Y = Q.rowwise() - qbar.transpose();

  Eigen::Matrix2d H = X.transpose() * Y;
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat2 U = svd.matrixU();
  Mat2 V = svd.matrixV();

  Mat2 R = V * U.transpose();
  if (R.determinant() < 0.0) {
    V.col(1) *= -1.0;
    R = V * U.transpose();
  }

  Vec2 t = qbar - R * pbar;

  return Pose2{R, t};
}

double RMSE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
  Eigen::MatrixXd D = A - B;
  double s = 0.0;
  for (int i = 0; i < D.rows(); ++i) {
    s += D.row(i).squaredNorm();
  }
  return std::sqrt(s / static_cast<double>(D.rows()));
}

int main() {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> ux(-5.0, 5.0);
  std::uniform_real_distribution<double> uy(-3.0, 3.0);
  std::normal_distribution<double> n01(0.0, 1.0);

  const int N = 200;
  Eigen::MatrixXd P(N, 2);
  for (int i = 0; i < N; ++i) {
    P(i,0) = ux(rng);
    P(i,1) = uy(rng);
  }

  double theta_gt = 12.0 * M_PI / 180.0;
  Vec2 t_gt(1.2, -0.7);
  Mat2 R_gt = Rot2(theta_gt);

  double sigma = 0.02;
  Eigen::MatrixXd Q(N, 2);
  for (int i = 0; i < N; ++i) {
    Vec2 q = R_gt * P.row(i).transpose() + t_gt;
    q(0) += sigma * n01(rng);
    q(1) += sigma * n01(rng);
    Q.row(i) = q.transpose();
  }

  Pose2 est = Kabsch2D(P, Q);

  double theta_hat = std::atan2(est.R(1,0), est.R(0,0));

  std::cout << "theta_gt(deg) = " << theta_gt * 180.0 / M_PI
            << ", t_gt = [" << t_gt.transpose() << "]\n";
  std::cout << "theta_hat(deg)= " << theta_hat * 180.0 / M_PI
            << ", t_hat= [" << est.t.transpose() << "]\n";

  Eigen::MatrixXd Q_hat(N, 2);
  for (int i = 0; i < N; ++i) {
    Vec2 q = est.R * P.row(i).transpose() + est.t;
    Q_hat.row(i) = q.transpose();
  }

  std::cout << "RMSE(Q_hat, Q) = " << RMSE(Q_hat, Q) << "\n";
  return 0;
}
