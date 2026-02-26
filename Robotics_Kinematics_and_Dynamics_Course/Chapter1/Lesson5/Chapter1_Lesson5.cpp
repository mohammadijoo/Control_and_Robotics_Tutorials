#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix3d J;
    J << 0.1, 0.0, 0.0,
          0.0, 1.0, 0.999,
          0.0, 1.0, 1.001;

    // Approximate 2-norm condition number using SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(J);
    double sigma_max = svd.singularValues()(0);
    double sigma_min = svd.singularValues()(2);
    double condJ = sigma_max / sigma_min;

    std::cout << "cond(J) = " << condJ << std::endl;
    if (condJ > 1e6) {
        std::cout << "Warning: Jacobian is ill-conditioned" << std::endl;
    }

    Eigen::Vector3d v(0.0, 0.0, 1.0);
    Eigen::Vector3d dq = J.colPivHouseholderQr().solve(v);
    std::cout << "dq = " << dq.transpose() << std::endl;

    return 0;
}
      
