#include <iostream>
#include <Eigen/Dense>

Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& R,
                              const Eigen::Vector3d& p) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0, 0) = R;
    T.block<3,1>(0, 3) = p;
    return T;
}

Eigen::Matrix4d inverseTransform(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3,3>(0, 0);
    Eigen::Vector3d p = T.block<3,1>(0, 3);
    Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
    Tinv.block<3,3>(0, 0) = R.transpose();
    Tinv.block<3,1>(0, 3) = -R.transpose() * p;
    return Tinv;
}

Eigen::Vector3d transformPoint(const Eigen::Matrix4d& T,
                               const Eigen::Vector3d& x) {
    Eigen::Vector4d x_h;
    x_h << x(0), x(1), x(2), 1.0;
    Eigen::Vector4d y_h = T * x_h;
    return y_h.head<3>();
}

int main() {
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 4.0,
                                          Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d p(0.5, 0.0, 0.0);

    Eigen::Matrix4d T = makeTransform(R, p);

    Eigen::Vector3d x(0.1, 0.0, 0.0);
    Eigen::Vector3d y = transformPoint(T, x);

    std::cout << "y = " << y.transpose() << std::endl;

    return 0;
}
      
