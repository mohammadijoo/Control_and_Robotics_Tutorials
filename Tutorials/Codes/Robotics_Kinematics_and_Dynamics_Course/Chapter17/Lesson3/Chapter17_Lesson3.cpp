#include <Eigen/Dense>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/// Assemble centroidal momentum matrix A(q) from per-link inertias and Jacobians.
MatrixXd assembleCentroidalMatrix(
    const std::vector<MatrixXd>& I_list,  // each 6x6
    const std::vector<MatrixXd>& J_list   // each 6 x nq
) {
    assert(I_list.size() == J_list.size());
    const std::size_t N = I_list.size();
    const int nq = static_cast<int>(J_list[0].cols());

    MatrixXd A = MatrixXd::Zero(6, nq);
    for (std::size_t i = 0; i < N; ++i) {
        A.noalias() += I_list[i] * J_list[i];
    }
    return A;
}

VectorXd centroidalMomentum(
    const std::vector<MatrixXd>& I_list,
    const std::vector<MatrixXd>& J_list,
    const VectorXd& qdot
) {
    MatrixXd A = assembleCentroidalMatrix(I_list, J_list);
    return A * qdot;  // 6x1 vector h_G
}

// Example usage with a generic rigid-body library:
// 1. Traverse kinematic tree;
// 2. For each link i, compute spatial inertia I_i^G and Jacobian J_i^G(q);
// 3. Call centroidalMomentum(I_list, J_list, qdot).
      
