#include <iostream>
#include <complex>
#include <vector>
#include <Eigen/Dense>

// Example: F(s) = 1 / (s*(s+3)) with distinct real poles
// Goal: find A, B such that F(s) = A/s + B/(s+3)

std::complex<double> F(std::complex<double> s) {
    return 1.0 / (s * (s + 3.0));
}

int main() {
    using cd = std::complex<double>;

    // Choose two distinct sample points s0, s1 (avoid poles)
    cd s0(0.5, 0.0);
    cd s1(1.0, 0.0);

    Eigen::Matrix<cd, 2, 2> M;
    Eigen::Matrix<cd, 2, 1> b;

    // F(s) = A/s + B/(s+3)
    // So F(s_i) = A/s_i + B/(s_i + 3)
    M(0, 0) = 1.0 / s0;
    M(0, 1) = 1.0 / (s0 + 3.0);
    b(0, 0) = F(s0);

    M(1, 0) = 1.0 / s1;
    M(1, 1) = 1.0 / (s1 + 3.0);
    b(1, 0) = F(s1);

    Eigen::Matrix<cd, 2, 1> x = M.colPivHouseholderQr().solve(b);
    cd A = x(0, 0);
    cd B = x(1, 0);

    std::cout << "A = " << A << std::endl;
    std::cout << "B = " << B << std::endl;

    // In embedded robotics controllers, such coefficients can be precomputed offline
    // and the time-domain response evaluated using exponentials e^{p_k t}.
    return 0;
}
