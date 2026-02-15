#include <iostream>
#include <Eigen/Dense>

int main() {
  const int n_joints = 12;
  const int ndof = 6 + n_joints;

  Eigen::VectorXd q(ndof);
  Eigen::VectorXd v(ndof);
  Eigen::MatrixXd H(ndof, ndof);

  q.setZero();
  v.setZero();
  H.setIdentity(); // Example only; real H(q) from dynamics library

  // Example base twist
  v.segment<6>(0) << 0.0, 0.0, 0.5, 0.1, 0.0, 0.0;
  // Example joint velocities
  v.segment(6, n_joints).setConstant(0.2);

  // Generalized momentum
  Eigen::VectorXd p = H * v;

  std::cout << "v = " << v.transpose() << std::endl;
  std::cout << "p = " << p.transpose() << std::endl;

  return 0;
}
      
