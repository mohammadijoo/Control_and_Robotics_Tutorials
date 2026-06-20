#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Dense>

using Wrench = Eigen::Matrix<double, 6, 1>;
using GraspMap = Eigen::Matrix<double, 6, Eigen::Dynamic>;

Wrench randomWrench(std::mt19937 &gen) {
    std::normal_distribution<double> dist(0.0, 1.0);
    Wrench w;
    for (int i = 0; i < 6; ++i) {
        w(i) = dist(gen);
    }
    double nrm = w.norm();
    if (nrm > 1e-9) {
        w /= nrm;
    }
    return w;
}

GraspMap buildGraspMap(std::size_t numContacts,
                       std::size_t raysPerContact,
                       std::mt19937 &gen) {
    std::size_t p = numContacts * raysPerContact;
    GraspMap G(6, static_cast<int>(p));
    std::size_t col = 0;
    for (std::size_t i = 0; i < numContacts; ++i) {
        for (std::size_t r = 0; r < raysPerContact; ++r) {
            G.col(static_cast<int>(col)) = randomWrench(gen);
            ++col;
        }
    }
    return G;
}

double epsilonQuality(const GraspMap &G, std::size_t numDirs,
                      std::mt19937 &gen) {
    std::normal_distribution<double> dist(0.0, 1.0);
    double eps = std::numeric_limits<double>::infinity();
    for (std::size_t k = 0; k < numDirs; ++k) {
        Wrench v;
        for (int i = 0; i < 6; ++i) {
            v(i) = dist(gen);
        }
        double nrm = v.norm();
        if (nrm > 1e-9) v /= nrm;
        Eigen::RowVectorXd vT = v.transpose();
        Eigen::RowVectorXd proj = vT * G;  // 1 x p
        double support = proj.maxCoeff();
        if (support < eps) eps = support;
    }
    return std::max(0.0, eps);
}

int main() {
    std::mt19937 gen(42);
    GraspMap G = buildGraspMap(4, 4, gen);
    double q = epsilonQuality(G, 200, gen);
    std::cout << "Approximate epsilon quality: " << q << std::endl;
    return 0;
}
      
