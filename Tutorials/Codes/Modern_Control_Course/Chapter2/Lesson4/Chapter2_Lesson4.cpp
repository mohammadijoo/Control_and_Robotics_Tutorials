#include <iostream>
#include <Eigen/Dense>

Eigen::MatrixXd similarityTransform(const Eigen::MatrixXd& A,
                                    const Eigen::MatrixXd& T) {
    // Prefer solving linear systems rather than forming inverse explicitly:
    // A_tilde = T^{-1} A T  =>  solve(T, (A*T))
    Eigen::MatrixXd AT = A * T;
    Eigen::MatrixXd A_tilde = T.fullPivLu().solve(AT);
    return A_tilde;
}

int main() {
    Eigen::MatrixXd A(2,2);
    A << 2.0, 1.0,
         0.0, 3.0;

    Eigen::MatrixXd T(2,2);
    T << 1.0, 1.0,
         0.0, 1.0;

    Eigen::MatrixXd A_tilde = similarityTransform(A, T);

    std::cout << "A:\n" << A << "\n\n";
    std::cout << "A_tilde:\n" << A_tilde << "\n\n";
    std::cout << "trace(A) = " << A.trace()
              << ", trace(A_tilde) = " << A_tilde.trace() << "\n";
    std::cout << "det(A) = " << A.determinant()
              << ", det(A_tilde) = " << A_tilde.determinant() << "\n";
    return 0;
}
      
