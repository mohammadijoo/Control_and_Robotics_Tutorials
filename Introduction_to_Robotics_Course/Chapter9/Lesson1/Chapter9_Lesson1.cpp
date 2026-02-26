#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix2d R_BA;
    R_BA << 0, 1,
           -1, 0;

    Eigen::Vector2d v_A(2.0, 1.0);
    Eigen::Vector2d v_B = R_BA * v_A;

    std::cout << "v_A = " << v_A.transpose() << std::endl;
    std::cout << "v_B = " << v_B.transpose() << std::endl;
    std::cout << "lengths: " << v_A.norm() << " " << v_B.norm() << std::endl;
    return 0;
}
