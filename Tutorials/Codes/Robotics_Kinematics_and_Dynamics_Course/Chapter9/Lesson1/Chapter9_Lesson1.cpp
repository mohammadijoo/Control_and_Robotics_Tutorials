#include <iostream>
#include <Eigen/Dense>

using Wrench6d = Eigen::Matrix<double, 6, 1>;

Eigen::Matrix3d hat(const Eigen::Vector3d& p) {
    Eigen::Matrix3d P;
    P <<   0.0,    -p.z(),  p.y(),
            p.z(),   0.0,   -p.x(),
           -p.y(),   p.x(),  0.0;
    return P;
}

Wrench6d transformWrench(
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& p,
    const Wrench6d& w_B)
{
    Eigen::Vector3d f_B = w_B.segment<3>(0);
    Eigen::Vector3d m_B = w_B.segment<3>(3);

    Eigen::Vector3d f_A = R * f_B;
    Eigen::Vector3d m_A = hat(p) * f_A + R * m_B;

    Wrench6d w_A;
    w_A.segment<3>(0) = f_A;
    w_A.segment<3>(3) = m_A;
    return w_A;
}

int main() {
    Eigen::Matrix3d R;
    R << 0.0, 0.0, 1.0,
          0.0, 1.0, 0.0,
         -1.0, 0.0, 0.0;
    Eigen::Vector3d p(0.5, 0.0, 0.0);

    Wrench6d w_B;
    w_B << 0.0, 0.0, 10.0,  // force
            0.0, 0.0,  0.0;  // moment

    Wrench6d w_A = transformWrench(R, p, w_B);
    std::cout << "w_A = \n" << w_A << std::endl;
    return 0;
}
      
