// Chapter13_Lesson1.cpp
// System Dynamics — Chapter 13, Lesson 1
// Modeling Two- and Multi-Degree-of-Freedom Mechanical Systems.
//
// Build M, C, K for a 2-DOF mass–spring–damper system, convert to state space,
// and integrate with a simple fixed-step RK4.
//
// Dependencies:
//   - Eigen (header-only): https://eigen.tuxfamily.org
//
// Compile (example):
//   g++ -O2 -std=c++17 Chapter13_Lesson1.cpp -I /path/to/eigen -o ch13_l1
//
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct TwoDOFParams {
    double m1 = 1.0, m2 = 0.8;
    double k1 = 200.0, k2 = 150.0, k3 = 100.0;
    double c1 = 1.5,   c2 = 1.0,   c3 = 0.8;
};

void two_dof_mck(const TwoDOFParams& p, MatrixXd& M, MatrixXd& C, MatrixXd& K) {
    M = MatrixXd::Zero(2,2);
    M(0,0) = p.m1; M(1,1) = p.m2;

    C = MatrixXd(2,2);
    C << (p.c1 + p.c2), -p.c2,
         -p.c2, (p.c2 + p.c3);

    K = MatrixXd(2,2);
    K << (p.k1 + p.k2), -p.k2,
         -p.k2, (p.k2 + p.k3);
}

void state_space_from_mck(const MatrixXd& M, const MatrixXd& C, const MatrixXd& K,
                          MatrixXd& A, MatrixXd& B) {
    const int n = static_cast<int>(M.rows());
    MatrixXd Minv = M.inverse();

    MatrixXd Z = MatrixXd::Zero(n,n);
    MatrixXd I = MatrixXd::Identity(n,n);

    A = MatrixXd::Zero(2*n, 2*n);
    A.block(0,0,n,n) = Z;
    A.block(0,n,n,n) = I;
    A.block(n,0,n,n) = -Minv * K;
    A.block(n,n,n,n) = -Minv * C;

    B = MatrixXd::Zero(2*n, n);
    B.block(n,0,n,n) = Minv;
}

// Fixed-step RK4 for xdot = A x + B f(t)
VectorXd rk4_step(const MatrixXd& A, const MatrixXd& B,
                  const VectorXd& x, double t, double h,
                  const std::function<VectorXd(double)>& force) {

    auto rhs = [&](double tau, const VectorXd& z) {
        return A * z + B * force(tau);
    };

    VectorXd k1 = rhs(t, x);
    VectorXd k2 = rhs(t + 0.5*h, x + 0.5*h*k1);
    VectorXd k3 = rhs(t + 0.5*h, x + 0.5*h*k2);
    VectorXd k4 = rhs(t + h, x + h*k3);
    return x + (h/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

int main() {
    TwoDOFParams p;
    MatrixXd M, C, K;
    two_dof_mck(p, M, C, K);

    MatrixXd A, B;
    state_space_from_mck(M, C, K, A, B);

    // Free response: f(t)=0, initial x=[q1,q2,q1dot,q2dot]
    auto f_free = [](double /*t*/) {
        VectorXd f(2); f << 0.0, 0.0; return f;
    };

    double t0 = 0.0, tf = 8.0, h = 0.001;
    int N = static_cast<int>((tf - t0)/h);

    VectorXd x(4);
    x << 0.02, -0.01, 0.0, 0.0;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "t,q1,q2\n";
    double t = t0;
    for (int i = 0; i <= N; ++i) {
        if (i % 200 == 0) { // print every 0.2 s
            std::cout << t << "," << x(0) << "," << x(1) << "\n";
        }
        x = rk4_step(A, B, x, t, h, f_free);
        t += h;
    }
    return 0;
}
