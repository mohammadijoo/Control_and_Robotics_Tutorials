#include <Eigen/Dense>
#include <iostream>

int main() {
  Eigen::Matrix3d R_AB;
  R_AB = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ());

  Eigen::Vector3d p_AB(1.0, 2.0, 0.0);

  Eigen::Matrix4d T_AB = Eigen::Matrix4d::Identity();
  T_AB.block<3,3>(0,0) = R_AB;
  T_AB.block<3,1>(0,3) = p_AB;

  Eigen::Vector4d x_B_h(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4d x_A_h = T_AB * x_B_h;

  std::cout << "x_A = " << x_A_h.head<3>().transpose() << std::endl;

  Eigen::Matrix4d T_BA = Eigen::Matrix4d::Identity();
  T_BA.block<3,3>(0,0) = R_AB.transpose();
  T_BA.block<3,1>(0,3) = -R_AB.transpose() * p_AB;

  std::cout << "T_AB*T_BA =\n" << (T_AB*T_BA) << std::endl;
  return 0;
}
