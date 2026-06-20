// Chapter26_Lesson5.cpp
// Servo design by state augmentation using Eigen and Ackermann's formula.
// Build example: g++ -std=c++17 Chapter26_Lesson5.cpp -I /path/to/eigen -O2 -o servo_aug

#include <Eigen/Dense>
#include <iostream>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd controllabilityMatrix(const MatrixXd& A, const MatrixXd& B) {
    const int n = A.rows();
    const int m = B.cols();
    MatrixXd W(n, n * m);
    MatrixXd Ak = MatrixXd::Identity(n, n);
    for (int k = 0; k < n; ++k) {
        W.block(0, k * m, n, m) = Ak * B;
        Ak = Ak * A;
    }
    return W;
}

std::vector<double> polynomialFromRoots(const std::vector<double>& roots) {
    // Returns coefficients of prod_i (s - roots[i]) in descending powers.
    std::vector<double> c{1.0};
    for (double r : roots) {
        std::vector<double> next(c.size() + 1, 0.0);
        for (std::size_t i = 0; i < c.size(); ++i) {
            next[i] += c[i];
            next[i + 1] += -r * c[i];
        }
        c = next;
    }
    return c;
}

MatrixXd matrixPolynomial(const MatrixXd& A, const std::vector<double>& coeff) {
    // coeff = [1, a_{n-1}, ..., a0], evaluates A^n + a_{n-1} A^(n-1) + ... + a0 I.
    const int n = A.rows();
    MatrixXd phi = MatrixXd::Zero(n, n);
    for (int i = 0; i <= n; ++i) {
        int power = n - i;
        MatrixXd Apow = MatrixXd::Identity(n, n);
        for (int k = 0; k < power; ++k) Apow = Apow * A;
        phi += coeff[i] * Apow;
    }
    return phi;
}

int main() {
    MatrixXd A(2, 2), B(2, 1), C(1, 2);
    A << 0.0, 1.0,
        -2.0, -0.8;
    B << 0.0,
         1.0;
    C << 1.0, 0.0;

    const int n = 2;
    const int p = 1;
    MatrixXd Aa = MatrixXd::Zero(n + p, n + p);
    MatrixXd Ba = MatrixXd::Zero(n + p, 1);
    MatrixXd Ea = MatrixXd::Zero(n + p, 1);

    Aa.block(0, 0, n, n) = A;
    Aa.block(n, 0, p, n) = -C;
    Ba.block(0, 0, n, 1) = B;
    Ea(n, 0) = 1.0;

    MatrixXd Wc = controllabilityMatrix(Aa, Ba);
    std::cout << "det(Wc) = " << Wc.determinant() << "\n";

    std::vector<double> desiredPoles{-2.0, -2.5, -3.0};
    std::vector<double> coeff = polynomialFromRoots(desiredPoles);
    MatrixXd phiA = matrixPolynomial(Aa, coeff);

    Eigen::RowVectorXd enT(3);
    enT << 0.0, 0.0, 1.0;
    Eigen::RowVectorXd Kaug = enT * Wc.inverse() * phiA;

    Eigen::RowVector2d Kx = Kaug.segment<2>(0);
    double Ki = -Kaug(2);

    std::cout << "Kaug = " << Kaug << "\n";
    std::cout << "Kx = " << Kx << "\n";
    std::cout << "Ki = " << Ki << "\n";

    MatrixXd Acl = Aa - Ba * Kaug;
    VectorXd z = VectorXd::Zero(3);
    const double r = 1.0;
    const double dt = 0.001;
    const double tf = 8.0;

    for (int k = 0; k < static_cast<int>(tf / dt); ++k) {
        VectorXd zdot = Acl * z + Ea.col(0) * r;
        z += dt * zdot;
    }

    double y = (C * z.head(2))(0, 0);
    double u = -(Kx * z.head(2))(0, 0) + Ki * z(2);
    std::cout << "Final y = " << y << "\n";
    std::cout << "Final tracking error = " << r - y << "\n";
    std::cout << "Final u = " << u << "\n";
    return 0;
}
