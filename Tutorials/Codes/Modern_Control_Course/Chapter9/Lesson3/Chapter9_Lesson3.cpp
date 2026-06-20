// Chapter9_Lesson3.cpp
// Modes, modal decomposition, and dominant modes for x_dot = A x.
//
// Dependency: Eigen C++ template library
// Ubuntu/Debian: sudo apt install libeigen3-dev
// Compile:
//   g++ -std=c++17 Chapter9_Lesson3.cpp -I /usr/include/eigen3 -O2 -o Chapter9_Lesson3
//
// The program compares full modal reconstruction with a dominant-mode
// approximation for a third-order continuous-time LTI system.

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <complex>
#include <iomanip>
#include <iostream>
#include <vector>
#include <algorithm>

using Complex = std::complex<double>;
using CMatrix = Eigen::MatrixXcd;
using CVector = Eigen::VectorXcd;

CVector modalState(
    const Eigen::VectorXcd& lambda,
    const CMatrix& V,
    const CMatrix& W,
    const CVector& x0,
    double t,
    const std::vector<int>& selectedModes
) {
    CVector z0 = W * x0;
    CVector x = CVector::Zero(x0.size());

    for (int idx : selectedModes) {
        x += V.col(idx) * std::exp(lambda(idx) * t) * z0(idx);
    }
    return x;
}

int main() {
    Eigen::Matrix3d A;
    A << -0.25,  1.50, 0.0,
         -1.50, -0.25, 0.0,
          0.00,  0.00, -3.0;

    Eigen::RowVector3d C;
    C << 1.0, 0.0, 0.4;

    Eigen::Vector3d x0real;
    x0real << 1.0, -0.2, 2.0;

    Eigen::ComplexEigenSolver<Eigen::Matrix3d> solver(A);
    Eigen::VectorXcd lambda = solver.eigenvalues();
    CMatrix V = solver.eigenvectors();
    CMatrix W = V.inverse();

    CVector x0 = x0real.cast<Complex>();

    std::cout << "Eigenvalues:\n";
    for (int i = 0; i < lambda.size(); ++i) {
        std::cout << "  lambda_" << i << " = "
                  << lambda(i).real() << " + "
                  << lambda(i).imag() << "j\n";
    }

    std::vector<int> allModes = {0, 1, 2};

    std::vector<int> dominant = allModes;
    std::sort(dominant.begin(), dominant.end(), [&](int a, int b) {
        return lambda(a).real() > lambda(b).real();
    });
    dominant.resize(2);

    std::cout << "\nDominant mode indices:";
    for (int idx : dominant) {
        std::cout << " " << idx;
    }
    std::cout << "\n\n";

    Eigen::RowVectorXcd Cc = C.cast<Complex>();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "t, y_full, y_dominant, absolute_error\n";

    for (int k = 0; k <= 40; ++k) {
        double t = 0.5 * k;

        CVector xFull = modalState(lambda, V, W, x0, t, allModes);
        CVector xDom = modalState(lambda, V, W, x0, t, dominant);

        Complex yFull = (Cc * xFull)(0);
        Complex yDom = (Cc * xDom)(0);
        double error = std::abs(yFull.real() - yDom.real());

        std::cout << t << ", "
                  << yFull.real() << ", "
                  << yDom.real() << ", "
                  << error << "\n";
    }

    return 0;
}
