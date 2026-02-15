#include <iostream>
#include <vector>
#include <complex>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXcd;

// Build companion matrix for monic polynomial
// P(s) = s^n + a_{n-1} s^{n-1} + ... + a_0
MatrixXd companion(const std::vector<double>& a) {
    const std::size_t n = a.size();
    MatrixXd C = MatrixXd::Zero(n, n);
    for (std::size_t i = 0; i + 1 < n; ++i) {
        C(i, i + 1) = 1.0;
    }
    for (std::size_t i = 0; i < n; ++i) {
        C(n - 1, i) = -a[i];
    }
    return C;
}

int main() {
    // Example: P(s, K) = s^3 + 7 s^2 + 10 s + K
    double K_min = 0.0, K_max = 80.0;
    int numK = 50;

    for (int j = 0; j <= numK; ++j) {
        double K = K_min + (K_max - K_min) * j / static_cast<double>(numK);
        // coefficients a_0, a_1, a_2 for s^0, s^1, s^2
        std::vector<double> a = {K, 10.0, 7.0};  // [a0, a1, a2]
        MatrixXd C = companion(a);
        Eigen::EigenSolver<MatrixXd> es(C);
        VectorXcd eigvals = es.eigenvalues();

        std::cout << "K = " << K << " poles: ";
        for (int i = 0; i < eigvals.size(); ++i) {
            std::cout << eigvals[i] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
