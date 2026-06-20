// Chapter16_Lesson4.cpp
// Continuous–Discrete Conversions: Zero-Order Hold, Exact Discretization
// C++ implementation using Eigen (and unsupported MatrixFunctions for exp())

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <fstream>
#include <iostream>
#include <vector>

struct DiscreteModel {
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
};

DiscreteModel exactDiscretizeZOH(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double Ts) {
    const int n = static_cast<int>(A.rows());
    const int m = static_cast<int>(B.cols());

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n + m, n + m);
    M.block(0, 0, n, n) = A;
    M.block(0, n, n, m) = B;

    Eigen::MatrixXd Md = (M * Ts).exp();  // matrix exponential
    DiscreteModel dm;
    dm.Ad = Md.block(0, 0, n, n);
    dm.Bd = Md.block(0, n, n, m);
    return dm;
}

DiscreteModel eulerDiscretize(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double Ts) {
    DiscreteModel dm;
    dm.Ad = Eigen::MatrixXd::Identity(A.rows(), A.cols()) + Ts * A;
    dm.Bd = Ts * B;
    return dm;
}

int main() {
    // Mass-spring-damper example
    const double m = 1.0, c = 0.6, k = 4.0, b = 1.0;
    Eigen::MatrixXd A(2, 2);
    Eigen::MatrixXd B(2, 1);
    Eigen::MatrixXd C(1, 2);
    Eigen::MatrixXd D(1, 1);

    A << 0.0, 1.0,
        -k / m, -c / m;
    B << 0.0,
         b / m;
    C << 1.0, 0.0;
    D << 0.0;

    const double Ts = 0.1;

    DiscreteModel exact = exactDiscretizeZOH(A, B, Ts);
    DiscreteModel euler = eulerDiscretize(A, B, Ts);

    std::cout << "Ad (exact ZOH):\n" << exact.Ad << "\n\n";
    std::cout << "Bd (exact ZOH):\n" << exact.Bd << "\n\n";
    std::cout << "Ad (Euler):\n" << euler.Ad << "\n\n";
    std::cout << "Bd (Euler):\n" << euler.Bd << "\n\n";

    // Simulate with a step input (u[k] = 0 for k<5, then 1)
    const int N = 120;
    Eigen::VectorXd xExact = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd xEuler = Eigen::VectorXd::Zero(2);

    std::ofstream csv("Chapter16_Lesson4_output.csv");
    csv << "k,t,u,y_exact,y_euler,x1_exact,x2_exact\n";

    for (int kstep = 0; kstep < N; ++kstep) {
        double u = (kstep >= 5) ? 1.0 : 0.0;

        double yExact = (C * xExact)(0, 0) + D(0, 0) * u;
        double yEuler = (C * xEuler)(0, 0) + D(0, 0) * u;

        csv << kstep << "," << (kstep * Ts) << "," << u << ","
            << yExact << "," << yEuler << ","
            << xExact(0) << "," << xExact(1) << "\n";

        xExact = exact.Ad * xExact + exact.Bd * u;
        xEuler = euler.Ad * xEuler + euler.Bd * u;
    }

    csv.close();
    std::cout << "Simulation data written to Chapter16_Lesson4_output.csv\n";
    return 0;
}
