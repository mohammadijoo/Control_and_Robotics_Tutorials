// Chapter12_Lesson4.cpp
// Robust pose-graph style constraints using Ceres Solver robust losses (Huber).
// This is a compact example illustrating how robust kernels appear in practice.
// Build (example):
//   mkdir build && cd build
//   cmake .. && cmake --build .
// Requires: Ceres Solver installed.

#include <ceres/ceres.h>
#include <cmath>
#include <iostream>
#include <vector>

static inline double WrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

struct RelativePose2dCost {
  RelativePose2dCost(double dx, double dy, double dth, double sxy, double sth)
      : dx_(dx), dy_(dy), dth_(dth), sxy_(sxy), sth_(sth) {}

  template <typename T>
  bool operator()(const T* const pi, const T* const pj, T* residuals) const {
    // pi = [xi, yi, thi], pj = [xj, yj, thj]
    const T c = ceres::cos(pi[2]);
    const T s = ceres::sin(pi[2]);
    const T dxw = pj[0] - pi[0];
    const T dyw = pj[1] - pi[1];

    // relative translation in i-frame
    const T xrel =  c * dxw + s * dyw;
    const T yrel = -s * dxw + c * dyw;

    // relative angle
    const T threl = ceres::atan2(ceres::sin(pj[2] - pi[2]), ceres::cos(pj[2] - pi[2]));

    residuals[0] = (xrel - T(dx_)) / T(sxy_);
    residuals[1] = (yrel - T(dy_)) / T(sxy_);
    residuals[2] = (threl - T(dth_)) / T(sth_);
    return true;
  }

  double dx_, dy_, dth_;
  double sxy_, sth_;
};

int main() {
  // Tiny graph: chain + one bad loop closure to show robustness.
  const int N = 15;
  std::vector<double> poses(3 * N, 0.0);

  auto Pose = [&](int i) { return &poses[3 * i]; };

  // Initialize along x with small yaw
  for (int k = 1; k < N; ++k) {
    Pose(k)[0] = Pose(k - 1)[0] + 0.3;
    Pose(k)[1] = 0.0;
    Pose(k)[2] = 0.02 * k;
  }

  const double sigma_xy = 0.05;
  const double sigma_th = 0.02;

  ceres::Problem problem;

  // Add all parameter blocks
  for (int i = 0; i < N; ++i) {
    problem.AddParameterBlock(Pose(i), 3);
  }
  // Fix first pose (gauge)
  problem.SetParameterBlockConstant(Pose(0));

  // Odometry edges (near-perfect synthetic)
  for (int k = 0; k < N - 1; ++k) {
    const double dx = 0.3;
    const double dy = 0.0;
    const double dth = 0.02;

    ceres::CostFunction* cost =
        new ceres::AutoDiffCostFunction<RelativePose2dCost, 3, 3, 3>(
            new RelativePose2dCost(dx, dy, dth, sigma_xy, sigma_th));

    // Robust kernel: Huber loss
    ceres::LossFunction* loss = new ceres::HuberLoss(1.5);

    problem.AddResidualBlock(cost, loss, Pose(k), Pose(k + 1));
  }

  // One bad loop closure (outlier)
  {
    const int i = 0;
    const int j = N - 1;
    const double dx_bad = 2.0;
    const double dy_bad = -1.0;
    const double dth_bad = 1.0;

    ceres::CostFunction* cost =
        new ceres::AutoDiffCostFunction<RelativePose2dCost, 3, 3, 3>(
            new RelativePose2dCost(dx_bad, dy_bad, dth_bad, sigma_xy, sigma_th));

    // The same robust kernel will downweight this constraint as residual grows.
    ceres::LossFunction* loss = new ceres::HuberLoss(1.5);

    problem.AddResidualBlock(cost, loss, Pose(i), Pose(j));
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 30;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "\nFinal pose of last node:\n";
  std::cout << "x=" << Pose(N - 1)[0] << " y=" << Pose(N - 1)[1]
            << " th=" << Pose(N - 1)[2] << "\n";

  return 0;
}
