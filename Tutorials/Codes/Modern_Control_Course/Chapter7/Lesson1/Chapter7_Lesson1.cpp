// ===== Code block 1 extracted from Chapter7/Lesson1.html =====
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

int main() {
    using Eigen::Matrix2d;
    using Eigen::Vector2d;

    Matrix2d A;
    A << 0.0, 1.0,
        -2.0, -3.0;

    double t0 = 0.0;
    double t  = 1.0;

    Vector2d x0;
    x0 << 1.0, 0.0;

    Matrix2d E = (A * (t - t0)).exp(); // matrix exponential
    Vector2d x = E * x0;

    std::cout << "exp(A*(t-t0)):\n" << E << "\n";
    std::cout << "x(t):\n" << x << "\n";
    return 0;
}
      

// ===== Code block 2 extracted from Chapter7/Lesson1.html =====
#include <iostream>
#include <Eigen/Dense>

Eigen::MatrixXd expm_series(const Eigen::MatrixXd& A, double t, int K=30) {
    const int n = A.rows();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd At = A * t;

    Eigen::MatrixXd term = I;
    Eigen::MatrixXd S = I;

    for (int k = 1; k <= K; ++k) {
        term = term * (At / double(k)); // term = At^k / k!
        S += term;
    }
    return S;
}

int main() {
    Eigen::Matrix2d A;
    A << 0.0, 1.0,
        -2.0, -3.0;

    double t = 1.0;
    Eigen::Matrix2d E = expm_series(A, t, 40);
    std::cout << "Series exp(A t):\n" << E << "\n";
    return 0;
}
      
