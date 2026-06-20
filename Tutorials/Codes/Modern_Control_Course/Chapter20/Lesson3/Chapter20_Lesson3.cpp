// Chapter20_Lesson3.cpp
// Internal vs external equivalence of continuous-time LTI systems.
// Dependency: Eigen 3 (header-only). Example compile command:
// g++ -std=c++17 Chapter20_Lesson3.cpp -I /path/to/eigen -O2 -o Chapter20_Lesson3

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <vector>

using Eigen::MatrixXd;

MatrixXd ctrb(const MatrixXd& A, const MatrixXd& B) {
    int n = A.rows();
    MatrixXd Ctrb(n, n * B.cols());
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        Ctrb.block(0, k * B.cols(), n, B.cols()) = Ak * B;
        Ak = A * Ak;
    }
    return Ctrb;
}

MatrixXd obsv(const MatrixXd& A, const MatrixXd& C) {
    int n = A.rows();
    MatrixXd Obsv(n * C.rows(), n);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        Obsv.block(k * C.rows(), 0, C.rows(), n) = C * Ak;
        Ak = Ak * A;
    }
    return Obsv;
}

int rankOf(const MatrixXd& M, double tol = 1e-10) {
    Eigen::FullPivLU<MatrixXd> lu(M);
    lu.setThreshold(tol);
    return lu.rank();
}

MatrixXd transferValue(const MatrixXd& A, const MatrixXd& B,
                       const MatrixXd& C, const MatrixXd& D, double s) {
    int n = A.rows();
    MatrixXd resolvent = (s * MatrixXd::Identity(n, n) - A).inverse();
    return C * resolvent * B + D;
}

std::vector<MatrixXd> markovParameters(const MatrixXd& A, const MatrixXd& B,
                                       const MatrixXd& C, const MatrixXd& D,
                                       int count = 6) {
    int n = A.rows();
    std::vector<MatrixXd> params;
    params.push_back(D);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < count; ++k) {
        params.push_back(C * Ak * B);
        Ak = A * Ak;
    }
    return params;
}

void report(const std::string& name, const MatrixXd& A, const MatrixXd& B,
            const MatrixXd& C, const MatrixXd& D) {
    int n = A.rows();
    std::cout << "\n" << name << "\n";
    std::cout << "A =\n" << A << "\nB =\n" << B << "\nC =\n" << C << "\nD =\n" << D << "\n";
    std::cout << "rank controllability = " << rankOf(ctrb(A, B)) << " of " << n << "\n";
    std::cout << "rank observability   = " << rankOf(obsv(A, C)) << " of " << n << "\n";
    std::cout << "minimal? " << ((rankOf(ctrb(A, B)) == n && rankOf(obsv(A, C)) == n) ? "yes" : "no") << "\n";
}

int main() {
    MatrixXd A1(2, 2), B1(2, 1), C1(1, 2), D1(1, 1);
    A1 << -1.0, 0.0,
           0.0,-2.0;
    B1 << 1.0,
          1.0;
    C1 << 1.0, 1.0;
    D1 << 0.0;

    MatrixXd T(2, 2);
    T << 1.0, 2.0,
         0.5, 1.5;
    MatrixXd A2 = T * A1 * T.inverse();
    MatrixXd B2 = T * B1;
    MatrixXd C2 = C1 * T.inverse();
    MatrixXd D2 = D1;

    MatrixXd A3(3, 3), B3(3, 1), C3(1, 3), D3(1, 1);
    A3 << -1.0, 0.0, 0.0,
           0.0,-2.0, 0.0,
           0.0, 0.0, 5.0;
    B3 << 1.0, 1.0, 0.0;
    C3 << 1.0, 1.0, 0.0;
    D3 << 0.0;

    report("Sigma_1 minimal", A1, B1, C1, D1);
    report("Sigma_2 internally equivalent to Sigma_1", A2, B2, C2, D2);
    report("Sigma_3 externally equivalent but nonminimal", A3, B3, C3, D3);

    std::cout << "\nInternal-equivalence residuals for Sigma_1 and Sigma_2:\n";
    std::cout << "A residual = " << (A2 - T * A1 * T.inverse()).norm() << "\n";
    std::cout << "B residual = " << (B2 - T * B1).norm() << "\n";
    std::cout << "C residual = " << (C2 - C1 * T.inverse()).norm() << "\n";
    std::cout << "D residual = " << (D2 - D1).norm() << "\n";

    std::cout << "\nTransfer-function samples G_i(s):\n";
    for (double s : {0.1, 1.0, 3.0, 10.0}) {
        std::cout << std::fixed << std::setprecision(6)
                  << "s=" << s
                  << "  G1=" << transferValue(A1, B1, C1, D1, s)(0, 0)
                  << "  G2=" << transferValue(A2, B2, C2, D2, s)(0, 0)
                  << "  G3=" << transferValue(A3, B3, C3, D3, s)(0, 0)
                  << "\n";
    }

    std::cout << "\nFirst Markov parameters for Sigma_1 and Sigma_3:\n";
    auto m1 = markovParameters(A1, B1, C1, D1);
    auto m3 = markovParameters(A3, B3, C3, D3);
    for (size_t k = 0; k < m1.size(); ++k) {
        std::cout << "k=" << k << "  Sigma_1=" << m1[k](0, 0)
                  << "  Sigma_3=" << m3[k](0, 0) << "\n";
    }
    return 0;
}
