#include <Eigen/Dense>

using namespace Eigen;

Matrix3d hatOmega(const Vector3d &w) {
    Matrix3d W;
    W << 0.0,   -w(2),  w(1),
          w(2),  0.0,   -w(0),
         -w(1),  w(0),   0.0;
    return W;
}

Vector3d veeOmega(const Matrix3d &W) {
    return Vector3d(W(2, 1), W(0, 2), W(1, 0));
}

Matrix3d so3Exp(const Vector3d &omega, double theta) {
    double n = omega.norm();
    if (n < 1e-9) {
        return Matrix3d::Identity();
    }
    Vector3d w = omega / n;
    Matrix3d W = hatOmega(w);
    Matrix3d W2 = W * W;
    return Matrix3d::Identity()
           + std::sin(theta) * W
           + (1.0 - std::cos(theta)) * W2;
}

void so3Log(const Matrix3d &R, Vector3d &omega, double &theta) {
    double cos_theta = (R.trace() - 1.0) / 2.0;
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    theta = std::acos(cos_theta);
    if (theta < 1e-9) {
        omega = Vector3d::Zero();
        theta = 0.0;
        return;
    }
    Matrix3d W = (theta / (2.0 * std::sin(theta))) * (R - R.transpose());
    omega = veeOmega(W);
    omega.normalize();
}

Matrix4d se3Exp(const Vector3d &v, const Vector3d &omega, double theta) {
    Matrix4d T = Matrix4d::Identity();
    double n = omega.norm();
    if (n < 1e-9) {
        // pure translation
        T.block<3, 1>(0, 3) = v * theta;
        return T;
    }
    Vector3d w = omega / n;
    Matrix3d W = hatOmega(w);
    Matrix3d W2 = W * W;
    Matrix3d R = so3Exp(w, theta);
    Matrix3d I = Matrix3d::Identity();
    Matrix3d J = I * theta
                 + (1.0 - std::cos(theta)) * W
                 + (theta - std::sin(theta)) * W2;
    Vector3d p = J * v;

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;
    return T;
}

void se3Log(const Matrix4d &T, Vector3d &v, Vector3d &omega, double &theta) {
    Matrix3d R = T.block<3, 3>(0, 0);
    Vector3d p = T.block<3, 1>(0, 3);

    so3Log(R, omega, theta);
    if (theta < 1e-9) {
        // pure translation
        theta = p.norm();
        if (theta < 1e-9) {
            v.setZero();
            omega.setZero();
            theta = 0.0;
            return;
        }
        v = p / theta;
        omega.setZero();
        return;
    }

    Matrix3d W = hatOmega(omega);
    Matrix3d W2 = W * W;
    Matrix3d I = Matrix3d::Identity();
    Matrix3d Jinv = I
                    - 0.5 * W
                    + (1.0 / (theta * theta)
                       - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta)))
                      * W2;
    v = Jinv * p;
}
      
