#include <iostream>
#include <Eigen/Core>
#include <Eigen/Polynomial>

int main() {
    // Example: G(s) = 1 / (s^2 + 3 s + 2)
    Eigen::VectorXd den(3);
    den << 1.0, 3.0, 2.0;  // coefficients of s^2 + 3 s + 2

    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(den);

    auto roots = solver.roots();
    bool asymptoticallyStable = true;

    for (int i = 0; i < roots.size(); ++i) {
        double realPart = roots[i].real();
        std::cout << "pole[" << i << "] = " << roots[i] << "\n";
        if (realPart >= 0.0) {
            asymptoticallyStable = false;
        }
    }

    if (asymptoticallyStable) {
        std::cout << "System is asymptotically stable.\n";
    } else {
        std::cout << "System is NOT asymptotically stable.\n";
    }

    return 0;
}
