/*
Chapter14_Lesson1.cpp

Kalman observability matrix and rank condition using Eigen.

Compile example:
  g++ -std=c++17 Chapter14_Lesson1.cpp -I /path/to/eigen -O2 -o Chapter14_Lesson1
*/

#include <Eigen/Dense>
#include <iostream>
#include <string>

Eigen::MatrixXd observabilityMatrix(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C) {
    const int n = static_cast<int>(A.rows());
    const int p = static_cast<int>(C.rows());

    Eigen::MatrixXd O(p * n, n);
    Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(n, n);

    for (int k = 0; k < n; ++k) {
        O.block(k * p, 0, p, n) = C * Ak;
        Ak = Ak * A;
    }
    return O;
}

int numericalRank(const Eigen::MatrixXd& M, double* toleranceOut = nullptr) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();

    double maxSingular = (s.size() > 0) ? s(0) : 0.0;
    double tol = std::max(M.rows(), M.cols()) * std::numeric_limits<double>::epsilon() * maxSingular;

    int rank = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) {
            ++rank;
        }
    }

    if (toleranceOut != nullptr) {
        *toleranceOut = tol;
    }
    return rank;
}

Eigen::MatrixXd nullSpaceSVD(const Eigen::MatrixXd& M) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
    Eigen::VectorXd s = svd.singularValues();
    double tol = std::max(M.rows(), M.cols()) * std::numeric_limits<double>::epsilon() * (s.size() > 0 ? s(0) : 0.0);

    int rank = 0;
    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > tol) {
            ++rank;
        }
    }

    return svd.matrixV().rightCols(M.cols() - rank);
}

void analyzeObservability(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C, const std::string& name) {
    Eigen::MatrixXd O = observabilityMatrix(A, C);
    double tol = 0.0;
    int rank = numericalRank(O, &tol);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(O);
    std::cout << "\n" << name << "\n";
    std::cout << std::string(name.size(), '-') << "\n";
    std::cout << "A =\n" << A << "\n";
    std::cout << "C =\n" << C << "\n";
    std::cout << "Observability matrix O_n =\n" << O << "\n";
    std::cout << "Singular values = " << svd.singularValues().transpose() << "\n";
    std::cout << "Tolerance = " << tol << "\n";
    std::cout << "rank(O_n) = " << rank << " of n = " << A.rows() << "\n";
    std::cout << "Observable? " << (rank == A.rows() ? "true" : "false") << "\n";

    if (rank < A.rows()) {
        std::cout << "Basis for unobservable initial-state directions:\n";
        std::cout << nullSpaceSVD(O) << "\n";
    }
}

int main() {
    Eigen::MatrixXd A1(2, 2);
    A1 << 0.0, 1.0,
          0.0, 0.0;
    Eigen::MatrixXd C1(1, 2);
    C1 << 1.0, 0.0;
    analyzeObservability(A1, C1, "Example 1: observable double integrator");

    Eigen::MatrixXd A2(2, 2);
    A2 << 0.0, 0.0,
          0.0, -2.0;
    Eigen::MatrixXd C2(1, 2);
    C2 << 1.0, 0.0;
    analyzeObservability(A2, C2, "Example 2: unobservable second mode");

    return 0;
}
