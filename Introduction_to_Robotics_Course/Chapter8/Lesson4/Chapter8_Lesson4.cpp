#include <iostream>
#include <Eigen/Dense>

int main() {
  // Scalar weighted fusion
  Eigen::Vector3d z; z << 1.02, 0.97, 1.10;
  Eigen::Vector3d w; w << 4.0, 2.0, 1.0;
  double x_hat = (w.dot(z)) / w.sum();
  std::cout << "Scalar fused estimate: " << x_hat << std::endl;

  // Vector WLS fusion
  Eigen::Matrix2d H1 = Eigen::Matrix2d::Identity();
  Eigen::Vector2d z1; z1 << 2.0, 1.0;

  Eigen::RowVector2d H2; H2 << 1.0, 0.0;
  Eigen::VectorXd z2(1); z2 << 1.8;

  Eigen::Matrix2d W1 = 5.0 * Eigen::Matrix2d::Identity();
  Eigen::MatrixXd W2(1,1); W2 << 2.0;

  Eigen::Matrix2d A = H1.transpose()*W1*H1 + H2.transpose()*W2*H2;
  Eigen::Vector2d b = H1.transpose()*W1*z1 + H2.transpose()*W2*z2;

  Eigen::Vector2d x_hat_vec = A.ldlt().solve(b);
  std::cout << "Vector fused estimate:\n" << x_hat_vec << std::endl;
  return 0;
}
