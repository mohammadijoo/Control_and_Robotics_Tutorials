#include <vector>
#include <Eigen/Dense>

using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W << 0.0,    -w.z(),  w.y(),
          w.z(),  0.0,    -w.x(),
         -w.y(),  w.x(),   0.0;
    return W;
}

Matrix6d adjoint(const Matrix4d& T) {
    Matrix6d Ad = Matrix6d::Zero();
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d p = T.block<3,1>(0,3);
    Eigen::Matrix3d p_hat = skew(p);

    Ad.block<3,3>(0,0) = R;
    Ad.block<3,3>(3,0) = p_hat * R;
    Ad.block<3,3>(3,3) = R;
    return Ad;
}

std::vector<Vector6d> forwardVelocityChain(
    const std::vector<Matrix4d>& T_list,
    const std::vector<Vector6d>& S_list,
    const Eigen::VectorXd& qdot,
    const Vector6d& V0 = Vector6d::Zero())
{
    const std::size_t n = S_list.size();
    std::vector<Vector6d> V_links;
    V_links.reserve(n);

    Vector6d V_prev = V0;

    for (std::size_t i = 0; i < n; ++i) {
        const Matrix4d& T_i = T_list[i];
        Matrix4d T_inv = T_i.inverse();
        Matrix6d Ad_inv = adjoint(T_inv);
        Vector6d V_prev_in_i = Ad_inv * V_prev;
        Vector6d V_i = V_prev_in_i + S_list[i] * qdot(static_cast<int>(i));
        V_links.push_back(V_i);
        V_prev = V_i;
    }
    return V_links;
}
      
