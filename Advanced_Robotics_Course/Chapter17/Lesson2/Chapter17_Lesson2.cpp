#include <Eigen/Dense>
#include <vector>

struct DeformableRopeState {
  std::size_t n;
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> velocities;

  explicit DeformableRopeState(std::size_t n_nodes)
      : n(n_nodes),
        positions(n_nodes, Eigen::Vector3d::Zero()),
        velocities(n_nodes, Eigen::Vector3d::Zero()) {}

  Eigen::VectorXd q() const {
    Eigen::VectorXd q_vec(3 * n);
    for (std::size_t i = 0; i < n; ++i) {
      q_vec.segment<3>(3 * i) = positions[i];
    }
    return q_vec;
  }

  Eigen::VectorXd qdot() const {
    Eigen::VectorXd qd_vec(3 * n);
    for (std::size_t i = 0; i < n; ++i) {
      qd_vec.segment<3>(3 * i) = velocities[i];
    }
    return qd_vec;
  }

  Eigen::VectorXd stateVector() const {
    Eigen::VectorXd s(6 * n);
    s << q(), qdot();
    return s;
  }

  void fromStateVector(const Eigen::VectorXd& s) {
    assert(static_cast<std::size_t>(s.size()) == 6 * n);
    Eigen::VectorXd q_vec = s.head(3 * n);
    Eigen::VectorXd qd_vec = s.tail(3 * n);
    for (std::size_t i = 0; i < n; ++i) {
      positions[i] = q_vec.segment<3>(3 * i);
      velocities[i] = qd_vec.segment<3>(3 * i);
    }
  }
};

Eigen::MatrixXd stiffnessMatrixChain(std::size_t n_nodes, double k_spring) {
  Eigen::MatrixXd L = Eigen::MatrixXd::Zero(n_nodes, n_nodes);
  for (std::size_t i = 0; i + 1 < n_nodes; ++i) {
    L(i, i) += 1.0;
    L(i + 1, i + 1) += 1.0;
    L(i, i + 1) -= 1.0;
    L(i + 1, i) -= 1.0;
  }
  Eigen::MatrixXd K_scalar = k_spring * L;
  // Kronecker product with I_3
  Eigen::MatrixXd K = Eigen::kroneckerProduct(K_scalar, Eigen::Matrix3d::Identity());
  return K;
}
      
