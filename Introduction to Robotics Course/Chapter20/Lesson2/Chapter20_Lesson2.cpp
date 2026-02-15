#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Matrix2d A;
    A << 0.0, 1.0,
          -2.0, -0.5;

    Eigen::EigenSolver<Eigen::Matrix2d> es(A);
    Eigen::VectorXcd eigvals = es.eigenvalues();

    bool stable = true;
    for (int i = 0; i < eigvals.size(); ++i) {
        if (eigvals[i].real() >= 0.0) {
            stable = false;
        }
    }

    if (stable) {
        std::cout << "A is Hurwitz: SDR stability check PASSED\n";
    } else {
        std::cout << "A is NOT Hurwitz: SDR stability check FAILED\n";
    }
    return 0;
}
      
