/*
Chapter13_Lesson3.cpp
System Dynamics (Control Engineering) — Chapter 13, Lesson 3
Modal Coordinates and Decoupling of MDOF Systems (Undamped Case)

Build and run (example):
  g++ -O2 -std=c++17 Chapter13_Lesson3.cpp -I /path/to/eigen -o demo
  ./demo

This program:
1) Builds a 3-DOF mass-spring chain (M, K)
2) Solves K phi = lambda M phi (symmetric generalized EVP)
3) Mass-normalizes modes: Phi^T M Phi = I
4) Simulates decoupled modal ODEs with RK4: qdd + omega^2 q = Phi^T f(t)
5) Reconstructs x(t) = Phi q(t)

Dependency:
  Eigen (header-only)
*/
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static void massNormalize(MatrixXd& Phi, const MatrixXd& M) {
    for (int i = 0; i < Phi.cols(); ++i) {
        double mi = Phi.col(i).transpose() * M * Phi.col(i);
        Phi.col(i) /= std::sqrt(mi);
    }
}

static void build3DOF(MatrixXd& M, MatrixXd& K) {
    // wall --k1-- m1 --k2-- m2 --k3-- m3 --k4-- wall
    double m1 = 1.0, m2 = 1.2, m3 = 0.9;
    double k1 = 1200.0, k2 = 900.0, k3 = 700.0, k4 = 1100.0;

    M = MatrixXd::Zero(3,3);
    M(0,0)=m1; M(1,1)=m2; M(2,2)=m3;

    K = MatrixXd::Zero(3,3);
    K(0,0)=k1+k2; K(0,1)=-k2;
    K(1,0)=-k2;   K(1,1)=k2+k3; K(1,2)=-k3;
    K(2,1)=-k3;   K(2,2)=k3+k4;
}

static VectorXd force(double t, double F0, double Omega) {
    VectorXd f(3);
    f << F0*std::sin(Omega*t), 0.0, 0.0;
    return f;
}

// RK4 for second-order decoupled modal system written as first-order:
// z = [q; qd],  z' = [qd; -omega^2*q + p(t)]
static void rk4_step(VectorXd& q, VectorXd& qd,
                     const VectorXd& omega,
                     const VectorXd& p, double dt)
{
    const int n = q.size();
    auto acc = [&](const VectorXd& q_in, const VectorXd& p_in) {
        VectorXd qdd(n);
        for (int i = 0; i < n; ++i) qdd(i) = -omega(i)*omega(i)*q_in(i) + p_in(i);
        return qdd;
    };

    // k1
    VectorXd k1_q  = qd;
    VectorXd k1_qd = acc(q, p);

    // k2
    VectorXd q2  = q  + 0.5*dt*k1_q;
    VectorXd qd2 = qd + 0.5*dt*k1_qd;
    VectorXd k2_q  = qd2;
    VectorXd k2_qd = acc(q2, p);

    // k3
    VectorXd q3  = q  + 0.5*dt*k2_q;
    VectorXd qd3 = qd + 0.5*dt*k2_qd;
    VectorXd k3_q  = qd3;
    VectorXd k3_qd = acc(q3, p);

    // k4
    VectorXd q4  = q  + dt*k3_q;
    VectorXd qd4 = qd + dt*k3_qd;
    VectorXd k4_q  = qd4;
    VectorXd k4_qd = acc(q4, p);

    q  += (dt/6.0)*(k1_q  + 2.0*k2_q  + 2.0*k3_q  + k4_q);
    qd += (dt/6.0)*(k1_qd + 2.0*k2_qd + 2.0*k3_qd + k4_qd);
}

int main() {
    MatrixXd M, K;
    build3DOF(M, K);

    // Symmetric generalized EVP: K phi = lambda M phi
    Eigen::GeneralizedSelfAdjointEigenSolver<MatrixXd> ges(K, M);
    if (ges.info() != Eigen::Success) {
        std::cerr << "Eigen solve failed.\n";
        return 1;
    }

    VectorXd lambda = ges.eigenvalues();
    MatrixXd Phi = ges.eigenvectors();

    // omega = sqrt(lambda)
    VectorXd omega = lambda.cwiseMax(0.0).cwiseSqrt();

    // mass-normalize
    massNormalize(Phi, M);

    std::cout << "Natural frequencies (rad/s): " << omega.transpose() << "\n";
    std::cout << "Check Phi^T M Phi:\n" << (Phi.transpose()*M*Phi) << "\n";
    std::cout << "Check Phi^T K Phi:\n" << (Phi.transpose()*K*Phi) << "\n";

    // Forcing
    double F0 = 10.0;
    double Omega = 0.9 * omega(0); // near first mode

    // Initial conditions
    VectorXd q = VectorXd::Zero(3);
    VectorXd qd = VectorXd::Zero(3);

    // Time integration
    double t0 = 0.0, tf = 8.0, dt = 1e-3;
    int N = static_cast<int>((tf - t0)/dt);

    // Print a few samples for DOF1 displacement x1(t) = [Phi q]_1
    for (int step = 0; step <= N; ++step) {
        double t = t0 + step*dt;

        VectorXd f = force(t, F0, Omega);
        VectorXd p = Phi.transpose() * f; // modal force (mass-normalized)

        if (step % 1000 == 0) {
            VectorXd x = Phi * q;
            std::cout << "t=" << t << "  x1=" << x(0) << "  q1=" << q(0) << "\n";
        }

        rk4_step(q, qd, omega, p, dt);
    }

    return 0;
}
