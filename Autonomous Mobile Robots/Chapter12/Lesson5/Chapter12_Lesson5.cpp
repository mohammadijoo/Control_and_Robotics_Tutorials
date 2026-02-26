// Chapter12_Lesson5.cpp
// Lab: Build and Optimize a 2D Pose Graph (SE(2)) using Gauss-Newton / LM
//
// Dependencies: Eigen (header-only for dense), Eigen Sparse for sparse linear solve.
// Build (example):
//   g++ -O2 -std=c++17 Chapter12_Lesson5.cpp -I /path/to/eigen -o pose_graph
//
// Notes:
// - This is an educational from-scratch implementation.
// - For production, consider Ceres Solver, g2o, or GTSAM.

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

static inline double WrapAngle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  a -= M_PI;
  if (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

static inline Eigen::Matrix2d Rot2(double th) {
  const double c = std::cos(th), s = std::sin(th);
  Eigen::Matrix2d R;
  R << c, -s, s, c;
  return R;
}

static const Eigen::Matrix2d SKEW = (Eigen::Matrix2d() << 0.0, -1.0, 1.0, 0.0).finished();

struct EdgeSE2 {
  int i, j;
  Eigen::Vector3d z;     // [dx, dy, dtheta] in i frame
  Eigen::Matrix3d Omega; // information
};

static inline Eigen::Vector3d SE2Compose(const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
  Eigen::Matrix2d R = Rot2(x(2));
  Eigen::Vector2d t = x.head<2>() + R * y.head<2>();
  Eigen::Vector3d out;
  out << t(0), t(1), WrapAngle(x(2) + y(2));
  return out;
}

static inline Eigen::Vector3d SE2Inverse(const Eigen::Vector3d& x) {
  Eigen::Matrix2d R = Rot2(x(2));
  Eigen::Vector2d tinv = -(R.transpose() * x.head<2>());
  Eigen::Vector3d out;
  out << tinv(0), tinv(1), WrapAngle(-x(2));
  return out;
}

static inline Eigen::Vector3d SE2Between(const Eigen::Vector3d& xi, const Eigen::Vector3d& xj) {
  return SE2Compose(SE2Inverse(xi), xj);
}

static inline double PoseGraphCost(const Eigen::VectorXd& x, const std::vector<EdgeSE2>& edges) {
  double cost = 0.0;
  for (const auto& e : edges) {
    Eigen::Vector3d xi = x.segment<3>(3 * e.i);
    Eigen::Vector3d xj = x.segment<3>(3 * e.j);
    Eigen::Vector3d zhat = SE2Between(xi, xj);
    Eigen::Vector3d zinv = SE2Inverse(e.z);
    Eigen::Vector3d err = SE2Compose(zinv, zhat);
    err(2) = WrapAngle(err(2));
    cost += err.transpose() * e.Omega * err;
  }
  return cost;
}

static inline void LinearizeEdge(
    const Eigen::Vector3d& xi,
    const Eigen::Vector3d& xj,
    const Eigen::Vector3d& z,
    Eigen::Vector3d& e,
    Eigen::Matrix3d& A,
    Eigen::Matrix3d& B) {

  Eigen::Vector2d ti = xi.head<2>();
  Eigen::Vector2d tj = xj.head<2>();
  const double thi = xi(2);
  const double thj = xj(2);

  Eigen::Matrix2d Ri = Rot2(thi);
  Eigen::Matrix2d Rz = Rot2(z(2));
  Eigen::Vector2d dt = tj - ti;

  Eigen::Vector2d zhat_t = Ri.transpose() * dt;
  double zhat_th = WrapAngle(thj - thi);

  Eigen::Vector2d e_t = Rz.transpose() * (zhat_t - z.head<2>());
  double e_th = WrapAngle(zhat_th - z(2));

  e << e_t(0), e_t(1), e_th;

  A.setZero(); B.setZero();

  A.block<2,2>(0,0) = -Rz.transpose() * Ri.transpose();
  B.block<2,2>(0,0) =  Rz.transpose() * Ri.transpose();

  Eigen::Vector2d d_ri = -Ri.transpose() * (SKEW * dt);
  A.block<2,1>(0,2) = Rz.transpose() * d_ri;

  A(2,2) = -1.0;
  B(2,2) =  1.0;
}

static inline void BuildNormalEquations(
    const Eigen::VectorXd& x,
    const std::vector<EdgeSE2>& edges,
    int N,
    double lambda,
    Eigen::SparseMatrix<double>& H,
    Eigen::VectorXd& b) {

  const int dim = 3 * N;
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(edges.size() * 9 * 4);

  b.setZero(dim);

  auto addBlock = [&](int r0, int c0, const Eigen::Matrix3d& M) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        trips.emplace_back(r0 + r, c0 + c, M(r,c));
  };

  // Accumulate into temporary dense blocks per edge; later we sum by triplets (duplicates summed).
  for (const auto& edge : edges) {
    const int i = edge.i, j = edge.j;
    Eigen::Vector3d xi = x.segment<3>(3*i);
    Eigen::Vector3d xj = x.segment<3>(3*j);

    Eigen::Vector3d e;
    Eigen::Matrix3d A, Bm;
    LinearizeEdge(xi, xj, edge.z, e, A, Bm);

    const Eigen::Matrix3d& Om = edge.Omega;

    Eigen::Matrix3d Hii = A.transpose() * Om * A;
    Eigen::Matrix3d Hij = A.transpose() * Om * Bm;
    Eigen::Matrix3d Hji = Bm.transpose() * Om * A;
    Eigen::Matrix3d Hjj = Bm.transpose() * Om * Bm;

    Eigen::Vector3d bi = A.transpose() * Om * e;
    Eigen::Vector3d bj = Bm.transpose() * Om * e;

    addBlock(3*i, 3*i, Hii);
    addBlock(3*i, 3*j, Hij);
    addBlock(3*j, 3*i, Hji);
    addBlock(3*j, 3*j, Hjj);

    b.segment<3>(3*i) += bi;
    b.segment<3>(3*j) += bj;
  }

  H.resize(dim, dim);
  H.setFromTriplets(trips.begin(), trips.end());

  if (lambda > 0.0) {
    // LM: H + lambda * diag(H)
    Eigen::VectorXd d = H.diagonal();
    Eigen::SparseMatrix<double> D(dim, dim);
    std::vector<Eigen::Triplet<double>> dtr;
    dtr.reserve(dim);
    for (int k = 0; k < dim; ++k) dtr.emplace_back(k, k, d(k) + lambda * (d(k) + 1e-12));
    D.setFromTriplets(dtr.begin(), dtr.end());
    H = H + D - Eigen::SparseMatrix<double>(d.asDiagonal());
  }

  // Gauge fix: anchor node 0 => set first 3 variables to zero increment.
  for (int k = 0; k < 3; ++k) {
    int idx = k;
    for (Eigen::SparseMatrix<double>::InnerIterator it(H, idx); it; ++it) it.valueRef() = 0.0;
    // zero row idx by rebuilding is expensive; for simplicity add strong prior instead:
    // We'll do strong diagonal (equivalent to fixing).
  }
  // Strong prior (large weight) on node 0 increment.
  const double W = 1e12;
  std::vector<Eigen::Triplet<double>> prior;
  prior.reserve(3);
  prior.emplace_back(0,0,W);
  prior.emplace_back(1,1,W);
  prior.emplace_back(2,2,W);
  Eigen::SparseMatrix<double> P(dim, dim);
  P.setFromTriplets(prior.begin(), prior.end());
  H = H + P;
  b(0) = 0.0; b(1) = 0.0; b(2) = 0.0;
}

static Eigen::Matrix3d MakeInformation(double sig_xy, double sig_th) {
  Eigen::Matrix3d Cov = Eigen::Matrix3d::Zero();
  Cov(0,0) = sig_xy*sig_xy;
  Cov(1,1) = sig_xy*sig_xy;
  Cov(2,2) = sig_th*sig_th;
  return Cov.inverse();
}

static void SimulatePoseGraph(int N, Eigen::VectorXd& x0, std::vector<EdgeSE2>& edges, Eigen::VectorXd& gt) {
  std::mt19937 rng(4);
  std::normal_distribution<double> n01(0.0, 1.0);

  auto gauss = [&](double s){ return s * n01(rng); };

  gt.resize(3*N);
  Eigen::Vector3d x(0.0, 0.0, 0.0);
  for (int i = 0; i < N; ++i) {
    gt.segment<3>(3*i) = x;
    Eigen::Vector3d u(0.5, 0.0, 0.03);
    x = SE2Compose(x, u);
  }

  edges.clear();
  edges.reserve(2*N);

  // odometry edges
  Eigen::Matrix3d OmOdo = MakeInformation(0.05, 0.02);
  for (int i = 0; i < N-1; ++i) {
    Eigen::Vector3d xi = gt.segment<3>(3*i);
    Eigen::Vector3d xj = gt.segment<3>(3*(i+1));
    Eigen::Vector3d z = SE2Between(xi, xj);
    z(0) += gauss(0.05); z(1) += gauss(0.05); z(2) = WrapAngle(z(2) + gauss(0.02));
    edges.push_back({i, i+1, z, OmOdo});
  }

  // loop closures
  Eigen::Matrix3d OmLoop = MakeInformation(0.03, 0.01);
  for (int i = 0; i < N-12; i += 10) {
    int j = i + 10;
    Eigen::Vector3d xi = gt.segment<3>(3*i);
    Eigen::Vector3d xj = gt.segment<3>(3*j);
    Eigen::Vector3d z = SE2Between(xi, xj);
    z(0) += gauss(0.03); z(1) += gauss(0.03); z(2) = WrapAngle(z(2) + gauss(0.01));
    edges.push_back({i, j, z, OmLoop});
  }

  // initial guess from chaining odometry
  x0.resize(3*N);
  Eigen::Vector3d cur(0.0, 0.0, 0.0);
  x0.segment<3>(0) = cur;
  for (int i = 0; i < N-1; ++i) {
    cur = SE2Compose(cur, edges[i].z);
    x0.segment<3>(3*(i+1)) = cur;
  }
}

static void RMSE(const Eigen::VectorXd& x, const Eigen::VectorXd& gt, int N, double& pos, double& ang) {
  double sp = 0.0, sa = 0.0;
  for (int i = 0; i < N; ++i) {
    Eigen::Vector2d p = x.segment<2>(3*i);
    Eigen::Vector2d pg = gt.segment<2>(3*i);
    double d = (p - pg).norm();
    sp += d*d;
    double da = WrapAngle(x(3*i+2) - gt(3*i+2));
    sa += da*da;
  }
  pos = std::sqrt(sp / N);
  ang = std::sqrt(sa / N);
}

int main() {
  const int N = 60;
  Eigen::VectorXd x0, gt;
  std::vector<EdgeSE2> edges;
  SimulatePoseGraph(N, x0, edges, gt);

  double pos0, ang0;
  RMSE(x0, gt, N, pos0, ang0);
  std::cout << "Initial RMSE: pos=" << pos0 << " m, ang=" << ang0 << " rad\n";

  Eigen::VectorXd x = x0;
  double cost = PoseGraphCost(x, edges);
  std::cout << "iter 0: cost=" << cost << "\n";

  double lambda = 1e-3;
  const int maxIters = 25;
  for (int it = 1; it <= maxIters; ++it) {
    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd b(3*N);
    BuildNormalEquations(x, edges, N, lambda, H, b);

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(H);
    if (solver.info() != Eigen::Success) {
      std::cerr << "Cholesky failed.\n";
      break;
    }
    Eigen::VectorXd dx = solver.solve(-b);

    double step = dx.norm();
    Eigen::VectorXd xNew = x + dx;
    for (int i = 0; i < N; ++i) xNew(3*i+2) = WrapAngle(xNew(3*i+2));

    double newCost = PoseGraphCost(xNew, edges);
    if (newCost < cost) {
      x = xNew; cost = newCost;
      lambda = std::max(lambda / 3.0, 1e-9);
      std::cout << "iter " << it << ": cost=" << cost << ", step=" << step << ", lambda=" << lambda << " (accepted)\n";
    } else {
      lambda = std::min(lambda * 5.0, 1e9);
      std::cout << "iter " << it << ": rejected, lambda=" << lambda << "\n";
    }
    if (step < 1e-7) break;
  }

  double pos1, ang1;
  RMSE(x, gt, N, pos1, ang1);
  std::cout << "Optimized RMSE: pos=" << pos1 << " m, ang=" << ang1 << " rad\n";
  return 0;
}
