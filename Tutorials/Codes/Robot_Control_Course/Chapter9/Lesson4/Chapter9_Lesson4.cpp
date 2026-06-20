
#include <Eigen/Dense>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct LQRParams {
  std::vector<MatrixXd> A;
  std::vector<MatrixXd> B;
  MatrixXd Q;
  MatrixXd R;
  MatrixXd P_N;
};

struct LQRGains {
  std::vector<MatrixXd> K;
  std::vector<VectorXd> kff;  // feedforward (if needed)
};

LQRGains finiteHorizonLQR(const LQRParams& p) {
  const int N = static_cast<int>(p.A.size());
  const int nx = static_cast<int>(p.Q.rows());
  const int nu = static_cast<int>(p.R.rows());

  std::vector<MatrixXd> P(N + 1);
  P[N] = p.P_N;

  LQRGains gains;
  gains.K.resize(N);
  gains.kff.resize(N);

  for (int k = N - 1; k >= 0; --k) {
    const MatrixXd& Ak = p.A[k];
    const MatrixXd& Bk = p.B[k];
    const MatrixXd& Pkp1 = P[k + 1];

    MatrixXd S = p.R + Bk.transpose() * Pkp1 * Bk;
    MatrixXd Kk = -S.ldlt().solve(Bk.transpose() * Pkp1 * Ak);

    MatrixXd Pk = p.Q
      + Ak.transpose() * Pkp1 * Ak
      + Ak.transpose() * Pkp1 * Bk * Kk
      + Kk.transpose() * Bk.transpose() * Pkp1 * Ak
      + Kk.transpose() * p.R * Kk;

    gains.K[k] = Kk;
    gains.kff[k] = VectorXd::Zero(nu);
    P[k] = Pk;
  }

  return gains;
}

// In real-time loop:
// 1. Linearize dynamics to get A_k, B_k around current trajectory.
// 2. Call finiteHorizonLQR with modest N (e.g. N=20).
// 3. Apply u_k = K_k * (x_k - x_ref_k).
