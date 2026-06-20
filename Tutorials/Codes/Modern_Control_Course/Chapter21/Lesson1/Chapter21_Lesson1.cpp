// Chapter21_Lesson1.cpp
// Finite transmission-zero check using the Rosenbrock system matrix.
// Required library: Eigen 3
//
// Compile example:
//   g++ -std=c++17 Chapter21_Lesson1.cpp -I /path/to/eigen -O2 -o Chapter21_Lesson1
//
// The code verifies that z = -4 is a transmission zero for
// G(s) = (s + 4)/(s^2 + 3s + 2).

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <cmath>
#include <iostream>
#include <vector>

using Matrix = Eigen::MatrixXd;

Matrix rosenbrockMatrix(const Matrix& A, const Matrix& B, const Matrix& C,
                        const Matrix& D, double s) {
    const int n = static_cast<int>(A.rows());
    const int p = static_cast<int>(C.rows());
    const int m = static_cast<int>(B.cols());

    Matrix R(n + p, n + m);
    R.block(0, 0, n, n) = s * Matrix::Identity(n, n) - A;
    R.block(0, n, n, m) = -B;
    R.block(n, 0, p, n) = C;
    R.block(n, n, p, m) = D;
    return R;
}

int numericalRank(const Matrix& M, double tol = 1.0e-9) {
    Eigen::JacobiSVD<Matrix> svd(M);
    int r = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > tol) {
            ++r;
        }
    }
    return r;
}

double detRosenbrock(const Matrix& A, const Matrix& B, const Matrix& C,
                     const Matrix& D, double s) {
    Matrix R = rosenbrockMatrix(A, B, C, D, s);
    return R.determinant();
}

std::vector<double> scanRealZeros(const Matrix& A, const Matrix& B,
                                  const Matrix& C, const Matrix& D,
                                  double left, double right, double step) {
    std::vector<double> roots;

    double a = left;
    double fa = detRosenbrock(A, B, C, D, a);

    for (double b = left + step; b <= right; b += step) {
        double fb = detRosenbrock(A, B, C, D, b);

        if (std::abs(fa) < 1.0e-8) {
            roots.push_back(a);
        } else if (fa * fb < 0.0) {
            double lo = a;
            double hi = b;
            for (int k = 0; k < 80; ++k) {
                double mid = 0.5 * (lo + hi);
                double fm = detRosenbrock(A, B, C, D, mid);
                if (fa * fm <= 0.0) {
                    hi = mid;
                    fb = fm;
                } else {
                    lo = mid;
                    fa = fm;
                }
            }
            roots.push_back(0.5 * (lo + hi));
        }

        a = b;
        fa = fb;
    }

    return roots;
}

int main() {
    Matrix A(2, 2);
    A << 0.0, 1.0,
        -2.0, -3.0;

    Matrix B(2, 1);
    B << 0.0,
         1.0;

    Matrix C(1, 2);
    C << 4.0, 1.0;

    Matrix D(1, 1);
    D << 0.0;

    std::cout << "det R(s) scanned on the real line." << std::endl;
    auto roots = scanRealZeros(A, B, C, D, -20.0, 20.0, 0.1);
    for (double root : roots) {
        std::cout << "Candidate real transmission zero: " << root << std::endl;
    }

    Matrix Rminus4 = rosenbrockMatrix(A, B, C, D, -4.0);
    std::cout << "\nR(-4) =\n" << Rminus4 << std::endl;
    std::cout << "rank R(-4) = " << numericalRank(Rminus4) << std::endl;
    std::cout << "normal rank = " << A.rows() + B.cols() << std::endl;

    Matrix xu(3, 1);
    xu << 1.0, -4.0, 6.0;
    std::cout << "\nR(-4) * [x; u] =\n" << Rminus4 * xu << std::endl;

    return 0;
}
