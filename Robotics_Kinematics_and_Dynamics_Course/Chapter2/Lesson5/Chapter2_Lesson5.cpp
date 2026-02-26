#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

Matrix3d hat_so3(const Vector3d &w) {
    Matrix3d W;
    W <<    0.0, -w(2),  w(1),
          w(2),    0.0, -w(0),
         -w(1),  w(0),   0.0;
    return W;
}

Matrix4d hat_se3(const VectorXd &xi) {
    Vector3d w = xi.segment<3>(0);
    Vector3d v = xi.segment<3>(3);

    Matrix4d Xi = Matrix4d::Zero();
    Xi.block<3,3>(0,0) = hat_so3(w);
    Xi.block<3,1>(0,3) = v;
    return Xi;
}

Matrix4d exp_se3(const VectorXd &xi, double theta = 1.0) {
    Vector3d w = xi.segment<3>(0);
    Vector3d v = xi.segment<3>(3);
    double wnorm = w.norm();

    Matrix4d g = Matrix4d::Identity();
    if (wnorm < 1e-9) {
        // Pure translation
        g.block<3,1>(0,3) = v * theta;
        return g;
    }

    Vector3d what = w / wnorm;
    Matrix3d W = hat_so3(what);
    double th = theta * wnorm;

    Matrix3d I = Matrix3d::Identity();
    Matrix3d R = I + std::sin(th) * W + (1.0 - std::cos(th)) * (W * W);

    Matrix3d A = I * th + (1.0 - std::cos(th)) * W + (th - std::sin(th)) * (W * W);
    Vector3d p = A * v;

    g.block<3,3>(0,0) = R;
    g.block<3,1>(0,3) = p;
    return g;
}

Matrix<double,6,6> adjoint_SE3(const Matrix4d &g) {
    Matrix3d R = g.block<3,3>(0,0);
    Vector3d p = g.block<3,1>(0,3);
    Matrix3d Phat = hat_so3(p);

    Matrix<double,6,6> Ad = Matrix<double,6,6>::Zero();
    Ad.block<3,3>(0,0) = R;
    Ad.block<3,3>(3,0) = Phat * R;
    Ad.block<3,3>(3,3) = R;
    return Ad;
}

int main() {
    VectorXd xi(6);
    xi << 0.0, 0.0, 1.0, 0.1, 0.2, 0.0;

    Matrix4d g = exp_se3(xi, 0.5);
    std::cout << "g(theta):\\n" << g << std::endl;

    Matrix<double,6,6> Adg = adjoint_SE3(g);
    std::cout << "Ad_g:\\n" << Adg << std::endl;

    return 0;
}
      
