#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix3d;
using Eigen::Vector3d;

Matrix3d skew(const Vector3d& w) {
    Matrix3d K;
    K <<   0.0,   -w.z(),  w.y(),
            w.z(),  0.0,   -w.x(),
           -w.y(),  w.x(),  0.0;
    return K;
}

Matrix3d axisAngleToR(const Vector3d& axis, double theta) {
    Vector3d w = axis;
    double n = w.norm();
    if (n < 1e-12) {
        return Matrix3d::Identity();
    }
    w /= n;
    Matrix3d K = skew(w);
    Matrix3d I = Matrix3d::Identity();
    return I + std::sin(theta) * K + (1.0 - std::cos(theta)) * (K * K);
}

void RToAxisAngle(const Matrix3d& R, Vector3d& axis, double& theta) {
    double trace = R.trace();
    double cos_theta = (trace - 1.0) / 2.0;
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    theta = std::acos(cos_theta);

    const double eps = 1e-8;
    if (theta < eps) {
        axis = Vector3d(1.0, 0.0, 0.0);
        theta = 0.0;
        return;
    }

    axis.x() = (R(2, 1) - R(1, 2)) / (2.0 * std::sin(theta));
    axis.y() = (R(0, 2) - R(2, 0)) / (2.0 * std::sin(theta));
    axis.z() = (R(1, 0) - R(0, 1)) / (2.0 * std::sin(theta));
    axis.normalize();
}
      
