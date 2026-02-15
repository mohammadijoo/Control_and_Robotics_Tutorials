#include <iostream>
#include <vector>
#include <complex>
#include <Eigen/Dense>

// Construct companion matrix for a monic polynomial
// p(s) = s^n + a_{n-1} s^{n-1} + ... + a_0
Eigen::MatrixXd companion_from_coeffs(const std::vector<double> &a) {
    int n = static_cast<int>(a.size());
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n, n);
    // Last row: negative coefficients
    for (int i = 0; i < n; ++i) {
        C(n-1, i) = -a[i];
    }
    // Superdiagonal of ones
    for (int i = 0; i < n-1; ++i) {
        C(i, i+1) = 1.0;
    }
    return C;
}

// Evaluate closed-loop poles for a given K
std::vector<std::complex<double> >
closed_loop_poles(const std::vector<double> &den,
                  const std::vector<double> #,
                  double K) {
    int n = static_cast<int>(den.size()) - 1;
    // Pad numerator to same degree as denominator
    std::vector<double> num_pad(den.size(), 0.0);
    int shift = static_cast<int>(den.size()) - static_cast<int>(num.size());
    for (int i = 0; i < static_cast<int>(num.size()); ++i) {
        num_pad[i + shift] = num[i];
    }
    // Characteristic polynomial coefficients (monic assumed in den)
    std::vector<double> a(n);
    for (int i = 0; i < n; ++i) {
        // den[i+1] corresponds to s^{n-1-i}, similarly for num_pad
        a[i] = den[i+1] + K * num_pad[i+1];
    }
    Eigen::MatrixXd C = companion_from_coeffs(a);
    Eigen::EigenSolver<Eigen::MatrixXd> es(C);
    Eigen::VectorXcd eig = es.eigenvalues();
    std::vector<std::complex<double> > poles(eig.size());
    for (int i = 0; i < eig.size(); ++i) {
        poles[i] = eig[i];
    }
    return poles;
}

int main() {
    // G(s) = K (s+1) / [s (s+2) (s+4)]
    std::vector<double> num = {1.0, 1.0};         // s + 1
    std::vector<double> den = {1.0, 6.0, 8.0, 0.0}; // s^3 + 6 s^2 + 8 s

    for (double K : {0.0, 10.0, 50.0, 200.0}) {
        auto poles = closed_loop_poles(den, num, K);
        std::cout << "K = " << K << std::endl;
        for (auto &p : poles) {
            std::cout << "  pole = " << p << std::endl;
        }
    }
    return 0;
}
