#include <Eigen/Dense>
#include <vector>
#include <iostream>

using Vec3 = Eigen::Vector3d;
using Mat  = Eigen::MatrixXd;

Eigen::Matrix3d skew(const Vec3 &v) {
    Eigen::Matrix3d S;
    S <<  0.0,   -v.z(),  v.y(),
            v.z(),  0.0,   -v.x(),
           -v.y(),  v.x(),  0.0;
    return S;
}

Mat buildGraspMap(const std::vector<Vec3> &contacts) {
    const std::size_t m = contacts.size();
    Mat G_top(3, 3 * m);
    Mat G_bottom(3, 3 * m);
    G_top.setZero();
    G_bottom.setZero();

    for (std::size_t i = 0; i < m; ++i) {
        std::size_t idx = 3 * i;
        G_top.block<3,3>(0, idx) = Eigen::Matrix3d::Identity();
        G_bottom.block<3,3>(0, idx) = skew(contacts[i]);
    }
    Mat G(6, 3 * m);
    G << G_top,
         G_bottom;
    return G;
}

double epsilonSurrogate(const Mat &G) {
    Eigen::JacobiSVD<Mat> svd(G, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();
    return s.minCoeff();
}

int main() {
    std::vector<Vec3> contacts;
    contacts.emplace_back(0.0,  0.03, 0.0);
    contacts.emplace_back(0.0, -0.03, 0.0);

    Mat G = buildGraspMap(contacts);
    double q = epsilonSurrogate(G);
    std::cout << "sigma_min(G) = " << q << std::endl;
    return 0;
}
      
