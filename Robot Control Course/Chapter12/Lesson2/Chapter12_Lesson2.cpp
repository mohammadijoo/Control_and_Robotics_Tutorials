
#include <iostream>
#include <Eigen/Dense>

int main() {
  Eigen::Matrix2d Ad;
  Ad << 0.95, 0.01,
         -0.2, 0.90;

  Eigen::EigenSolver<Eigen::Matrix2d> es(Ad);
  Eigen::VectorXcd z = es.eigenvalues();

  std::cout << "Discrete poles: " << z.transpose() << std::endl;

  bool stable = true;
  for (int i = 0; i < z.size(); ++i) {
    if (std::abs(z[i]) >= 1.0) {
      stable = false;
    }
  }
  std::cout << "Schur-stable? " << (stable ? "yes" : "no") << std::endl;

  return 0;
}

// In a ROS control loop, Ad would come from offline design, and the loop would
// update x_{k+1} = Ad * x_k at each timer callback with a fixed period Ts.
