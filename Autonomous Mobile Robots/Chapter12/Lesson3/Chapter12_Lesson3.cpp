// Chapter12_Lesson3.cpp
// Pose Graph Optimization in SE(2) via Gauss-Newton (Eigen, sparse)
// Build (Linux/Mac):
//   g++ -O2 -std=c++17 Chapter12_Lesson3.cpp -I /path/to/eigen -o pg
// Run:
//   ./pg
//
// Notes:
// - This is an educational from-scratch optimizer (no Ceres/g2o).
// - For real systems, use Ceres Solver, g2o, or GTSAM.

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <iostream>
#include <vector>
#include <cmath>

struct EdgeSE2 {
  int i;
  int j;
  Eigen::Vector3d z;      // [dx, dy, dtheta] measurement (i->j in i-frame)
  Eigen::Matrix3d Omega;  // information
};

static inline double wrap_angle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0) a += 2.0 * M_PI;
  return a - M_PI;
}

static inline Eigen::Matrix2d rot2(double th) {
  double c = std::cos(th), s = std::sin(th);
  Eigen::Matrix2d R;
  R << c, -s,
       s,  c;
  return R;
}

// predict zhat = inv(xi) ⊕ xj
static inline Eigen::Vector3d predict_relative(const Eigen::Vector3d& xi,
                                               const Eigen::Vector3d& xj) {
  Eigen::Vector2d ti = xi.head<2>();
  Eigen::Vector2d tj = xj.head<2>();
  Eigen::Matrix2d Ri = rot2(xi(2));
  Eigen::Vector2d dt = Ri.transpose() * (tj - ti);
  double dth = wrap_angle(xj(2) - xi(2));
  return Eigen::Vector3d(dt(0), dt(1), dth);
}

// e = inv(z) ⊕ zhat (minimal translation+angle approximation)
static inline Eigen::Vector3d se2_error(const Eigen::Vector3d& xi,
                                        const Eigen::Vector3d& xj,
                                        const Eigen::Vector3d& z) {
  Eigen::Vector3d zhat = predict_relative(xi, xj);
  Eigen::Matrix2d Rz = rot2(z(2));
  Eigen::Vector2d terr = Rz.transpose() * (zhat.head<2>() - z.head<2>());
  double therr = wrap_angle(zhat(2) - z(2));
  return Eigen::Vector3d(terr(0), terr(1), therr);
}

static inline void se2_jacobians(const Eigen::Vector3d& xi,
                                 const Eigen::Vector3d& xj,
                                 const Eigen::Vector3d& z,
                                 Eigen::Matrix3d& A,
                                 Eigen::Matrix3d& B) {
  Eigen::Vector2d ti = xi.head<2>();
  Eigen::Vector2d tj = xj.head<2>();
  Eigen::Matrix2d Ri = rot2(xi(2));
  Eigen::Matrix2d Rz = rot2(z(2));
  Eigen::Matrix2d A2 = Rz.transpose() * Ri.transpose();

  Eigen::Matrix2d S;
  S << 0.0, -1.0,
       1.0,  0.0;

  Eigen::Vector2d dt = (tj - ti);

  A.setZero();
  B.setZero();

  A.block<2,2>(0,0) = -A2;
  B.block<2,2>(0,0) =  A2;

  Eigen::Vector2d dti_dtheta = Rz.transpose() * (-Ri.transpose() * (S * dt));
  A(0,2) = dti_dtheta(0);
  A(1,2) = dti_dtheta(1);

  A(2,2) = -1.0;
  B(2,2) =  1.0;
}

static inline void apply_increment(std::vector<Eigen::Vector3d>& X,
                                   const Eigen::VectorXd& dx,
                                   bool anchor_first_pose = true) {
  int N = (int)X.size();
  if (anchor_first_pose) {
    for (int k = 1; k < N; ++k) {
      Eigen::Vector3d d = dx.segment<3>(3*(k-1));
      X[k](0) += d(0);
      X[k](1) += d(1);
      X[k](2) = wrap_angle(X[k](2) + d(2));
    }
  } else {
    for (int k = 0; k < N; ++k) {
      Eigen::Vector3d d = dx.segment<3>(3*k);
      X[k](0) += d(0);
      X[k](1) += d(1);
      X[k](2) = wrap_angle(X[k](2) + d(2));
    }
  }
}

static inline void build_normal_equations(const std::vector<Eigen::Vector3d>& X,
                                          const std::vector<EdgeSE2>& edges,
                                          Eigen::SparseMatrix<double>& H,
                                          Eigen::VectorXd& b,
                                          double& chi2,
                                          bool anchor_first_pose = true) {
  int N = (int)X.size();
  int dim = anchor_first_pose ? 3*(N-1) : 3*N;

  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(edges.size() * 36); // rough

  b.setZero(dim);
  chi2 = 0.0;

  auto idx = [&](int pose_id) {
    return anchor_first_pose ? 3*(pose_id - 1) : 3*pose_id;
  };

  for (const auto& e : edges) {
    const auto& xi = X[e.i];
    const auto& xj = X[e.j];

    Eigen::Vector3d r = se2_error(xi, xj, e.z);
    Eigen::Matrix3d A, Bm;
    se2_jacobians(xi, xj, e.z, A, Bm);

    chi2 += r.transpose() * e.Omega * r;

    Eigen::Matrix3d AtOmega = A.transpose() * e.Omega;
    Eigen::Matrix3d BtOmega = Bm.transpose() * e.Omega;

    Eigen::Matrix3d Hii = AtOmega * A;
    Eigen::Matrix3d Hij = AtOmega * Bm;
    Eigen::Matrix3d Hji = BtOmega * A;
    Eigen::Matrix3d Hjj = BtOmega * Bm;

    Eigen::Vector3d bi = AtOmega * r;
    Eigen::Vector3d bj = BtOmega * r;

    auto addBlock = [&](int p, int q, const Eigen::Matrix3d& M) {
      if (anchor_first_pose && (p == 0 || q == 0)) return;
      int ip = idx(p), iq = idx(q);
      for (int a = 0; a < 3; ++a)
        for (int c = 0; c < 3; ++c)
          trips.emplace_back(ip + a, iq + c, M(a,c));
    };

    addBlock(e.i, e.i, Hii);
    addBlock(e.i, e.j, Hij);
    addBlock(e.j, e.i, Hji);
    addBlock(e.j, e.j, Hjj);

    if (!(anchor_first_pose && e.i == 0)) {
      int ii = idx(e.i);
      b.segment<3>(ii) += bi;
    }
    if (!(anchor_first_pose && e.j == 0)) {
      int jj = idx(e.j);
      b.segment<3>(jj) += bj;
    }
  }

  H.resize(dim, dim);
  H.setFromTriplets(trips.begin(), trips.end());
}

static inline void make_synthetic_pose_graph(int N,
                                             std::vector<Eigen::Vector3d>& X0,
                                             std::vector<EdgeSE2>& edges) {
  std::vector<Eigen::Vector3d> Xtrue(N);
  Xtrue[0].setZero();

  for (int k = 1; k < N; ++k) {
    Xtrue[k](0) = Xtrue[k-1](0) + 0.5 * std::cos(0.1 * k);
    Xtrue[k](1) = Xtrue[k-1](1) + 0.5 * std::sin(0.1 * k);
    Xtrue[k](2) = wrap_angle(0.05 * k);
  }

  double noise_xy = 0.02, noise_th = 0.01;
  Eigen::Matrix3d Sigma = Eigen::Matrix3d::Zero();
  Sigma(0,0) = noise_xy * noise_xy;
  Sigma(1,1) = noise_xy * noise_xy;
  Sigma(2,2) = noise_th * noise_th;

  Eigen::Matrix3d Omega = Sigma.inverse();

  auto randn = []() {
    // crude normal via Box-Muller
    static unsigned long long seed = 7ULL;
    seed = 6364136223846793005ULL * seed + 1ULL;
    double u1 = ((seed >> 11) * (1.0 / 9007199254740992.0));
    seed = 6364136223846793005ULL * seed + 1ULL;
    double u2 = ((seed >> 11) * (1.0 / 9007199254740992.0));
    double r = std::sqrt(-2.0 * std::log(std::max(1e-12, u1)));
    double th = 2.0 * M_PI * u2;
    return r * std::cos(th);
  };

  edges.clear();
  edges.reserve(N);

  for (int k = 0; k < N-1; ++k) {
    Eigen::Vector3d z = predict_relative(Xtrue[k], Xtrue[k+1]);
    z(0) += noise_xy * randn();
    z(1) += noise_xy * randn();
    z(2) = wrap_angle(z(2) + noise_th * randn());
    edges.push_back({k, k+1, z, Omega});
  }

  // loop closure 0 -> N-1
  Eigen::Vector3d zlc = predict_relative(Xtrue[0], Xtrue[N-1]);
  zlc(0) += noise_xy * randn();
  zlc(1) += noise_xy * randn();
  zlc(2) = wrap_angle(zlc(2) + noise_th * randn());
  edges.push_back({0, N-1, zlc, Omega});

  // initial guess: integrate noisy odometry only
  X0.assign(N, Eigen::Vector3d::Zero());
  for (int k = 0; k < N-1; ++k) {
    Eigen::Vector3d z = edges[k].z;
    Eigen::Matrix2d Rk = rot2(X0[k](2));
    X0[k+1].head<2>() = X0[k].head<2>() + Rk * z.head<2>();
    X0[k+1](2) = wrap_angle(X0[k](2) + z(2));
  }
}

int main() {
  std::vector<Eigen::Vector3d> X;
  std::vector<EdgeSE2> edges;

  make_synthetic_pose_graph(30, X, edges);

  std::cout << "Optimizing pose graph (Gauss-Newton, anchor pose 0)\n";

  int iters = 15;
  for (int it = 0; it < iters; ++it) {
    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd b;
    double chi2 = 0.0;

    int dim = 3 * ((int)X.size() - 1);
    b.resize(dim);

    build_normal_equations(X, edges, H, b, chi2, true);

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(H);
    if (solver.info() != Eigen::Success) {
      std::cerr << "Factorization failed\n";
      return 1;
    }
    Eigen::VectorXd dx = solver.solve(-b);

    apply_increment(X, dx, true);

    std::cout << "iter=" << it << "  chi2=" << chi2
              << "  |dx|=" << dx.norm() << "\n";

    if (dx.norm() < 1e-8) break;
  }

  std::cout << "\nFirst 5 poses (x,y,theta):\n";
  for (int k = 0; k < 5; ++k) {
    std::cout << "k=" << k << "  " << X[k].transpose() << "\n";
  }
  return 0;
}
