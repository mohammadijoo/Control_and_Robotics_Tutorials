#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix3d R_WB;
    R_WB << 0, -1, 0,
            1,  0, 0,
            0,  0, 1;

    Eigen::Vector3d t_WB(1.0, 2.0, 0.5);
    Eigen::Vector3d p_B(0.3, 0.0, 0.2);

    Eigen::Vector3d p_W = R_WB * p_B + t_WB;
    std::cout << "p_W = " << p_W.transpose() << std::endl;

    Eigen::Vector3d p_B_rec = R_WB.transpose() * (p_W - t_WB);
    std::cout << "p_B recovered = " << p_B_rec.transpose() << std::endl;
    return 0;
}
