#include <iostream>
#include <Eigen/Dense>
// If available in your Eigen installation:
// #include <unsupported/Eigen/MatrixFunctions>

Eigen::MatrixXd expm_series(const Eigen::MatrixXd& A, double t,
                            double tol = 1e-12, int max_terms = 200) {
    const int n = (int)A.rows();
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd term = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd At = A * t;

    for (int k = 1; k <= max_terms; ++k) {
        term = (term * At) / double(k);
        Eigen::MatrixXd S_new = S + term;
        double term_norm = term.norm();     // Frobenius norm
        double S_norm = S_new.norm();
        if (term_norm <= tol * S_norm) {
            return S_new;
        }
        S = S_new;
    }
    return S;
}

int main() {
    Eigen::MatrixXd A(2,2);
    A << 0.0, 1.0,
         -1.0, 0.0;
    double t = 0.7;

    Eigen::MatrixXd S = expm_series(A, t, 1e-14, 300);
    std::cout << "Series exp(A t):\n" << S << "\n\n";

    // If unsupported/Eigen/MatrixFunctions is available:
    // Eigen::MatrixXd S_lib = (A * t).exp();
    // std::cout << "Eigen exp(A t):\n" << S_lib << "\n";

    return 0;
}
