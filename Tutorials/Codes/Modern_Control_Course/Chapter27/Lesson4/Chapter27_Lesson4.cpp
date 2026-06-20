/*
Chapter27_Lesson4.cpp
State-space disturbance rejection using integral action.

Dependency:
    Eigen 3
Compile example:
    g++ -std=c++17 Chapter27_Lesson4.cpp -I /path/to/eigen -O2 -o Chapter27_Lesson4
*/

#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>

using Matrix3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;
using Row3 = Eigen::RowVector3d;

Matrix3 desiredPolynomialMatrix(const Matrix3& Aaug) {
    // Desired poles: -2, -2.5, -3
    // p(s) = s^3 + 7.5 s^2 + 18.5 s + 15
    return Aaug * Aaug * Aaug + 7.5 * Aaug * Aaug + 18.5 * Aaug + 15.0 * Matrix3::Identity();
}

Row3 ackermannGain(const Matrix3& Aaug, const Vector3& Baug) {
    Matrix3 ctrb;
    ctrb.col(0) = Baug;
    ctrb.col(1) = Aaug * Baug;
    ctrb.col(2) = Aaug * Aaug * Baug;

    Row3 last;
    last << 0.0, 0.0, 1.0;

    Matrix3 phiA = desiredPolynomialMatrix(Aaug);
    return last * ctrb.inverse() * phiA;
}

Vector3 dynamics(const Vector3& xa, const Row3& Kaug, double d0) {
    // Plant: x1_dot = x2, x2_dot = -2 x1 -0.6 x2 + u + d0, xi_dot = x1
    double x1 = xa(0);
    double x2 = xa(1);
    double xi = xa(2);
    double u = -(Kaug(0) * x1 + Kaug(1) * x2 + Kaug(2) * xi);

    Vector3 dx;
    dx << x2,
          -2.0 * x1 - 0.6 * x2 + u + d0,
          x1;
    return dx;
}

int main() {
    Matrix3 Aaug;
    Aaug << 0.0, 1.0, 0.0,
           -2.0, -0.6, 0.0,
            1.0, 0.0, 0.0;

    Vector3 Baug;
    Baug << 0.0, 1.0, 0.0;

    Row3 Kaug = ackermannGain(Aaug, Baug);
    std::cout << "Kaug = " << Kaug << std::endl;

    const double d0 = 0.5;
    const double h = 0.001;
    const double tf = 10.0;
    Vector3 xa;
    xa << 0.4, 0.0, 0.0;

    std::ofstream file("Chapter27_Lesson4_cpp_response.csv");
    file << "t,x1,x2,xi,u,y\n";

    for (int k = 0; k <= static_cast<int>(tf / h); ++k) {
        double t = k * h;
        double u = -(Kaug(0) * xa(0) + Kaug(1) * xa(1) + Kaug(2) * xa(2));
        double y = xa(0);
        file << std::setprecision(10) << t << "," << xa(0) << "," << xa(1) << ","
             << xa(2) << "," << u << "," << y << "\n";

        Vector3 k1 = dynamics(xa, Kaug, d0);
        Vector3 k2 = dynamics(xa + 0.5 * h * k1, Kaug, d0);
        Vector3 k3 = dynamics(xa + 0.5 * h * k2, Kaug, d0);
        Vector3 k4 = dynamics(xa + h * k3, Kaug, d0);
        xa += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }

    std::cout << "Final y = " << xa(0) << std::endl;
    std::cout << "CSV written to Chapter27_Lesson4_cpp_response.csv" << std::endl;
    return 0;
}
