
#include <iostream>
#include <Eigen/Dense>

int main() {
  double J_nom = 0.5;
  double Kp = 50.0;
  double Kd = 5.0;

  Eigen::Matrix2d A;
  A << 0.0, 1.0,
        -Kp / J_nom, -Kd / J_nom;

  Eigen::EigenSolver<Eigen::Matrix2d> es(A);
  Eigen::VectorXcd evals = es.eigenvalues();

  double alpha = -1e9;
  for (int i = 0; i < evals.size(); ++i) {
    double real_part = evals(i).real();
    alpha = std::max(alpha, real_part);
  }

  std::cout << "Eigenvalues: " << evals.transpose() << std::endl;
  std::cout << "Spectral abscissa alpha(A_cl): " << alpha << std::endl;

  if (alpha < 0.0) {
    std::cout << "Closed loop is (locally) asymptotically stable." << std::endl;
  } else {
    std::cout << "Closed loop is unstable or marginally stable." << std::endl;
  }

  return 0;
}
