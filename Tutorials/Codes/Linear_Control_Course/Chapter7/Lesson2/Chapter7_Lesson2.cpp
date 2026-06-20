#include <iostream>
#include <complex>
#include <Eigen/Dense>

std::string classifyEigenvalues(const Eigen::VectorXcd& evals) {
    bool hasPositive = false;
    bool hasImagAxisRepeated = false;
    int imagAxisCount = 0;

    for (int i = 0; i < evals.size(); ++i) {
        double real_part = evals(i).real();
        double imag_part = evals(i).imag();

        if (real_part > 0.0) {
            hasPositive = true;
        }
        if (std::abs(real_part) < 1e-9 && std::abs(imag_part) > 1e-9) {
            imagAxisCount++;
        }
    }

    if (hasPositive) return "unstable";
    if (imagAxisCount >= 1) {
        // Here we ignore multiplicity check for simplicity
        return "marginally stable or oscillatory";
    }
    return "asymptotically stable";
}

int main() {
    double J = 0.01;
    double b = 0.1;
    double k = 2.0;   // stiffness from PD-like position control

    Eigen::Matrix2d A;
    A << 0.0,              1.0,
          -k / J,   -b / J;

    Eigen::EigenSolver<Eigen::Matrix2d> es(A);
    Eigen::VectorXcd evals = es.eigenvalues();

    std::cout << "Eigenvalues of A:\n";
    for (int i = 0; i < evals.size(); ++i) {
        std::cout << evals(i) << std::endl;
    }

    std::cout << "Stability: " << classifyEigenvalues(evals) << std::endl;

    // In a ROS controller, this logic could be called whenever gains change
    // to ensure the joint-level closed-loop dynamics remain stable.
    return 0;
}
