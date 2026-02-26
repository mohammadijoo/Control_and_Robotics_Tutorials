#include <iostream>
#include <complex>
#include <Eigen/Dense>

// Degree of stability for P(s) = s^3 + 5 s^2 + 6 s + K
double degreeOfStability(double K) {
    // Companion matrix for s^3 + a2 s^2 + a1 s + a0
    double a2 = 5.0;
    double a1 = 6.0;
    double a0 = K;

    Eigen::Matrix3d A;
    A <<
        0.0, 0.0, -a0,
        1.0, 0.0, -a1,
        0.0, 1.0, -a2;

    Eigen::EigenSolver<Eigen::Matrix3d> solver(A);
    Eigen::VectorXcd eig = solver.eigenvalues();

    double alpha = std::numeric_limits<double>::infinity();
    for (int i = 0; i < eig.size(); ++i) {
        double realPart = eig[i].real();
        if (realPart >= 0.0) {
            // unstable
            return 0.0;
        }
        alpha = std::min(alpha, -realPart);
    }
    return alpha;
}

int main() {
    double K = 10.0;
    double alpha = degreeOfStability(K);
    if (alpha > 0.0) {
        std::cout << "Stable, alpha = " << alpha << std::endl;
    } else {
        std::cout << "Unstable" << std::endl;
    }

    // In a robotics controller, a similar computation could be used offline
    // to check stability margins of local joint loops before deployment.

    return 0;
}
