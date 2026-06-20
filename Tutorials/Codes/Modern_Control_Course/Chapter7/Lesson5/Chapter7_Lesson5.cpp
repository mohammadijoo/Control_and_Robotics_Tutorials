/*
Chapter7_Lesson5.cpp
Modern Control — Chapter 7, Lesson 5: Numerical Simulation of State Equations

Demonstrates:
  - Fixed-step RK4 simulation of x_dot = A x + B u(t)
  - Exact ZOH discretization via Van Loan method using matrix exponential

Dependencies:
  - Eigen (including unsupported MatrixFunctions module)

Build example (Linux/macOS):
  g++ -O2 -std=c++17 Chapter7_Lesson5.cpp -I /path/to/eigen -o sim

Note: Eigen's matrix exponential is in unsupported/Eigen/MatrixFunctions.
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

static double u_of_t(double t) {
    return std::sin(2.0 * t);
}

static Eigen::VectorXd f(double t, const Eigen::VectorXd& x,
                         const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    return A * x + B * Eigen::VectorXd::Constant(1, u_of_t(t));
}

static Eigen::VectorXd rk4_step(double t, const Eigen::VectorXd& x, double h,
                                const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    Eigen::VectorXd k1 = f(t, x, A, B);
    Eigen::VectorXd k2 = f(t + 0.5*h, x + 0.5*h*k1, A, B);
    Eigen::VectorXd k3 = f(t + 0.5*h, x + 0.5*h*k2, A, B);
    Eigen::VectorXd k4 = f(t + h, x + h*k3, A, B);
    return x + (h/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

static void van_loan_discretization(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double h,
                                    Eigen::MatrixXd& Ad, Eigen::MatrixXd& Bd) {
    // expm([A B; 0 0] h) = [Ad Bd; 0 I]
    const int n = static_cast<int>(A.rows());
    const int m = static_cast<int>(B.cols());
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n + m, n + m);
    M.block(0, 0, n, n) = A;
    M.block(0, n, n, m) = B;

    Eigen::MatrixXd E = (M * h).exp(); // matrix exponential
    Ad = E.block(0, 0, n, n);
    Bd = E.block(0, n, n, m);
}

int main() {
    Eigen::Matrix2d A;
    A << 0.0, 1.0,
        -2.0, -3.0;

    Eigen::Vector2d x0;
    x0 << 1.0, 0.0;

    Eigen::Matrix<double, 2, 1> B;
    B << 0.0,
         1.0;

    double t0 = 0.0, tf = 10.0, h = 0.01;
    int N = static_cast<int>(std::floor((tf - t0)/h));

    // RK4 simulation
    std::vector<double> ts(N + 1);
    std::vector<Eigen::Vector2d> xs(N + 1);
    ts[0] = t0;
    xs[0] = x0;

    double t = t0;
    Eigen::Vector2d x = x0;
    for (int k = 0; k < N; ++k) {
        x = rk4_step(t, x, h, A, B);
        t += h;
        ts[k+1] = t;
        xs[k+1] = x;
    }

    // Exact ZOH discretization
    Eigen::Matrix2d Ad;
    Eigen::Matrix<double, 2, 1> Bd;
    van_loan_discretization(A, B, h, Ad, Bd);

    Eigen::Vector2d xz = x0;
    for (int k = 0; k < N; ++k) {
        double tk = t0 + k*h;
        double uk = u_of_t(tk); // ZOH sample
        xz = Ad * xz + Bd * uk;
    }

    std::cout << "A_d (ZOH exact):\n" << Ad << "\n\n";
    std::cout << "B_d (ZOH exact):\n" << Bd << "\n\n";
    std::cout << "Final state RK4: " << xs.back().transpose() << "\n";
    std::cout << "Final state ZOH: " << xz.transpose() << "\n";
    return 0;
}
