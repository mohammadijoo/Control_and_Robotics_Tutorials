#include <Eigen/Dense>
#include <vector>
#include <iostream>

// Suppose you have an interface that logs joint configurations
// from your RBDL / Pinocchio simulations into an Eigen::MatrixXd.
Eigen::MatrixXd collectJointTrajectories(int K, int n);

struct PCABasis {
  Eigen::VectorXd q_mean;     // (n)
  Eigen::MatrixXd V_r;        // (n, r)
};

PCABasis computePCA(const Eigen::MatrixXd& Q, double energy_threshold = 0.99) {
  const int K = Q.rows();
  const int n = Q.cols();

  Eigen::VectorXd q_mean = Q.colwise().mean();     // (n)
  Eigen::MatrixXd Qc = Q.rowwise() - q_mean.transpose(); // center

  // SVD: Qc = U * S * V.transpose()
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      Qc, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd S = svd.singularValues();        // (min(K,n))
  Eigen::MatrixXd V = svd.matrixV();               // (n, n)

  // Compute energy and choose r
  Eigen::VectorXd S2 = S.array().square();
  double total = S2.sum();
  double cum = 0.0;
  int r = 0;
  for (int i = 0; i < S2.size(); ++i) {
    cum += S2(i);
    if (cum / total >= energy_threshold) {
      r = i + 1;
      break;
    }
  }
  if (r == 0) r = S2.size();

  Eigen::MatrixXd V_r = V.leftCols(r); // (n, r)

  PCABasis basis;
  basis.q_mean = q_mean;
  basis.V_r = V_r;
  return basis;
}

// Projection and reconstruction helpers
Eigen::VectorXd project(const Eigen::VectorXd& q, const PCABasis& basis) {
  // z = V_r^T (q - q_mean)
  return basis.V_r.transpose() * (q - basis.q_mean);
}

Eigen::VectorXd reconstruct(const Eigen::VectorXd& z, const PCABasis& basis) {
  // q = q_mean + V_r z
  return basis.q_mean + basis.V_r * z;
}

int main() {
  int K = 2000;
  int n = 12;
  Eigen::MatrixXd Q = collectJointTrajectories(K, n);
  PCABasis basis = computePCA(Q, 0.99);

  std::cout << "Reduced dimension r = "
            << basis.V_r.cols() << std::endl;

  // Example projection of a sample configuration:
  Eigen::VectorXd q = Q.row(100).transpose();
  Eigen::VectorXd z = project(q, basis);
  Eigen::VectorXd q_rec = reconstruct(z, basis);
  std::cout << "Reconstruction error norm = "
            << (q - q_rec).norm() << std::endl;
  return 0;
}
      
