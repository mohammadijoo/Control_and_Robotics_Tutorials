// Chapter9_Lesson2.cpp
// Eigenvalue-based stability criteria using the Eigen C++ library.
//
// Build example:
//   g++ -std=c++17 Chapter9_Lesson2.cpp -I /path/to/eigen -O2 -o Chapter9_Lesson2
//
// Eigen is header-only and is widely used for state-space computations.

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <complex>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using Matrix = Eigen::MatrixXd;
using Complex = std::complex<double>;

std::string classifyContinuous(const Matrix& A, double tol = 1e-10) {
    Eigen::EigenSolver<Matrix> solver(A);
    auto eigenvalues = solver.eigenvalues();

    double maxReal = -1.0e300;
    for (int i = 0; i < eigenvalues.size(); ++i) {
        maxReal = std::max(maxReal, eigenvalues[i].real());
    }

    if (maxReal < -tol) return "asymptotically stable";
    if (maxReal > tol) return "unstable";

    return "borderline: check semisimplicity of imaginary-axis eigenvalues";
}

std::string classifyDiscrete(const Matrix& A, double tol = 1e-10) {
    Eigen::EigenSolver<Matrix> solver(A);
    auto eigenvalues = solver.eigenvalues();

    double spectralRadius = 0.0;
    for (int i = 0; i < eigenvalues.size(); ++i) {
        spectralRadius = std::max(spectralRadius, std::abs(eigenvalues[i]));
    }

    if (spectralRadius < 1.0 - tol) return "asymptotically stable";
    if (spectralRadius > 1.0 + tol) return "unstable";

    return "borderline: check semisimplicity of unit-circle eigenvalues";
}

void printEigenReport(const std::string& name, const Matrix& A, bool continuous) {
    Eigen::EigenSolver<Matrix> solver(A);

    std::cout << "============================================================\n";
    std::cout << name << "\nA =\n" << A << "\n";
    std::cout << "Eigenvalues:\n";
    for (int i = 0; i < solver.eigenvalues().size(); ++i) {
        std::cout << "  " << solver.eigenvalues()[i] << "\n";
    }

    if (continuous) {
        std::cout << "Continuous-time classification: "
                  << classifyContinuous(A) << "\n";
    } else {
        std::cout << "Discrete-time classification: "
                  << classifyDiscrete(A) << "\n";
    }
}

int main() {
    Matrix A1(2, 2);
    A1 << -1.0, 0.0,
           0.0, -2.0;

    Matrix A2(2, 2);
    A2 << 0.0, -1.0,
           1.0,  0.0;

    Matrix A3(2, 2);
    A3 << 0.0, 1.0,
           0.0, 0.0;

    Matrix Ad1(2, 2);
    Ad1 << 0.8, 0.0,
            0.0, 0.5;

    printEigenReport("Continuous asymptotically stable example", A1, true);
    printEigenReport("Continuous oscillatory borderline example", A2, true);
    printEigenReport("Continuous defective zero-eigenvalue example", A3, true);
    printEigenReport("Discrete asymptotically stable example", Ad1, false);

    return 0;
}
