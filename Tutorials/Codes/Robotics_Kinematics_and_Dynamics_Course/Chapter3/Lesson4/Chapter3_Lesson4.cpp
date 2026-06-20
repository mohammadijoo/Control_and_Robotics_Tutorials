#include <Eigen/Dense>
#include <cmath>

using Matrix3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;

Matrix3 axisAngleToR(const Vector3& omega_hat, double theta) {
    Vector3 w = omega_hat.normalized();
    Eigen::Matrix3d K;
    K << 0.0,   -w.z(),  w.y(),
          w.z(),  0.0,   -w.x(),
         -w.y(),  w.x(),  0.0;
    return Matrix3::Identity()
         + std::sin(theta) * K
         + (1.0 - std::cos(theta)) * K * K;
}

void RToAxisAngle(const Matrix3& R, Vector3& omega_hat, double& theta) {
    double trace = R.trace();
    double cos_theta = (trace - 1.0) / 2.0;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    theta = std::acos(cos_theta);

    if (theta < 1e-8) {
        omega_hat = Vector3(1.0, 0.0, 0.0);
        theta = 0.0;
        return;
    }

    omega_hat = Vector3(
        R(2,1) - R(1,2),
        R(0,2) - R(2,0),
        R(1,0) - R(0,1)
    ) / (2.0 * std::sin(theta));
    omega_hat.normalize();
}

Quaternion RToQuat(const Matrix3& R) {
    Quaternion q;
    q = Quaternion(R);  // Eigen has built-in constructor
    q.normalize();
    return q;
}

Matrix3 quatToR(const Quaternion& q) {
    Quaternion qn = q.normalized();
    return qn.toRotationMatrix();
}

Matrix3 eulerZYXToR(double phi, double theta, double psi) {
    Matrix3 Rx, Ry, Rz;
    double c1 = std::cos(psi), s1 = std::sin(psi);
    double c2 = std::cos(theta), s2 = std::sin(theta);
    double c3 = std::cos(phi), s3 = std::sin(phi);

    Rz << c1, -s1, 0.0,
           s1,  c1, 0.0,
           0.0, 0.0, 1.0;
    Ry <<  c2, 0.0, s2,
            0.0, 1.0, 0.0,
           -s2, 0.0, c2;
    Rx << 1.0, 0.0, 0.0,
           0.0, c3, -s3,
           0.0, s3,  c3;

    return Rz * Ry * Rx;
}
      
