/*
Chapter19_Lesson3.cpp
Finite Difference (FD) and Finite Element (FE) Approximations (Conceptual Level)

This C++ example implements:
  1) 1D heat equation u_t = alpha u_xx via FD in space and theta-method in time.
  2) 1D Poisson -u'' = f(x) via linear FEM (Galerkin) on uniform mesh.

Dependencies (recommended):
  - Eigen (Dense + Sparse)
    https://eigen.tuxfamily.org/

Compile (example):
  g++ -O2 -std=c++17 Chapter19_Lesson3.cpp -I /path/to/eigen -o lesson19_3

Author: (course material generator)
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Sparse>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Triplet;

struct Grid1D {
    double L;
    int N; // nodes including boundaries
    std::vector<double> x;
    double dx;

    Grid1D(double L_, int N_) : L(L_), N(N_) {
        x.resize(N);
        for (int i = 0; i < N; ++i) x[i] = (L * i) / (N - 1);
        dx = x[1] - x[0];
    }
};

SparseMatrix<double> laplacian_dirichlet_1d(int N, double dx) {
    // interior unknowns: size n = N-2
    int n = N - 2;
    std::vector<Triplet<double>> T;
    T.reserve(3 * n);

    double s = 1.0 / (dx * dx);
    for (int i = 0; i < n; ++i) {
        T.emplace_back(i, i, -2.0 * s);
        if (i - 1 >= 0) T.emplace_back(i, i - 1, 1.0 * s);
        if (i + 1 < n)  T.emplace_back(i, i + 1, 1.0 * s);
    }
    SparseMatrix<double> A(n, n);
    A.setFromTriplets(T.begin(), T.end());
    return A;
}

void heat_fd_theta(
    const VectorXd& u0_full,
    double alpha,
    const Grid1D& grid,
    double dt,
    int steps,
    double theta,
    std::vector<double>& t_out,
    std::vector<VectorXd>& u_out
) {
    int N = grid.N;
    int n = N - 2;

    SparseMatrix<double> A = laplacian_dirichlet_1d(N, grid.dx);
    SparseMatrix<double> I(n, n);
    I.setIdentity();

    SparseMatrix<double> LHS = (I - theta * dt * alpha * A);
    SparseMatrix<double> RHS = (I + (1.0 - theta) * dt * alpha * A);

    // Dirichlet boundaries fixed
    double uL = u0_full(0);
    double uR = u0_full(N - 1);

    // interior vector
    VectorXd v = u0_full.segment(1, n);

    // Pre-factorize LHS
    Eigen::SparseLU<SparseMatrix<double>> solver;
    solver.analyzePattern(LHS);
    solver.factorize(LHS);

    t_out.clear(); u_out.clear();
    t_out.reserve(steps + 1);
    u_out.reserve(steps + 1);

    auto pack_full = [&](const VectorXd& vin) {
        VectorXd u(N);
        u.setZero();
        u(0) = uL;
        u(N - 1) = uR;
        u.segment(1, n) = vin;
        return u;
    };

    t_out.push_back(0.0);
    u_out.push_back(pack_full(v));

    VectorXd bc(n); bc.setZero();
    double s = 1.0 / (grid.dx * grid.dx);
    bc(0)     += uL * s;
    bc(n - 1) += uR * s;

    for (int k = 0; k < steps; ++k) {
        VectorXd rhs = RHS * v + dt * alpha * bc;
        v = solver.solve(rhs);

        t_out.push_back((k + 1) * dt);
        u_out.push_back(pack_full(v));
    }
}

std::pair<std::vector<double>, VectorXd> fem_poisson_p1(
    int Nel, double L,
    double (*f)(double),
    double u0, double uL
) {
    int Nn = Nel + 1;
    std::vector<double> x(Nn);
    double h = L / Nel;
    for (int i = 0; i < Nn; ++i) x[i] = i * h;

    MatrixXd K = MatrixXd::Zero(Nn, Nn);
    VectorXd F = VectorXd::Zero(Nn);

    for (int e = 0; e < Nel; ++e) {
        int i = e, j = e + 1;
        MatrixXd Ke(2,2);
        Ke <<  1.0, -1.0,
              -1.0,  1.0;
        Ke *= (1.0 / h);

        double xm = 0.5 * (x[i] + x[j]);
        double fe = f(xm);
        Eigen::Vector2d Fe;
        Fe << 1.0, 1.0;
        Fe *= (fe * (h / 2.0));

        K.block(i, i, 2, 2) += Ke;
        F.segment(i, 2) += Fe;
    }

    // Dirichlet elimination
    std::vector<int> free;
    free.reserve(Nn - 2);
    for (int i = 1; i < Nn - 1; ++i) free.push_back(i);

    // modify RHS
    F -= K.col(0) * u0;
    F -= K.col(Nn - 1) * uL;

    MatrixXd Kff(free.size(), free.size());
    VectorXd Ff(free.size());
    for (int a = 0; a < (int)free.size(); ++a) {
        Ff(a) = F(free[a]);
        for (int b = 0; b < (int)free.size(); ++b) {
            Kff(a,b) = K(free[a], free[b]);
        }
    }

    VectorXd uf = Kff.fullPivLu().solve(Ff);

    VectorXd u = VectorXd::Zero(Nn);
    u(0) = u0;
    u(Nn - 1) = uL;
    for (int a = 0; a < (int)free.size(); ++a) u(free[a]) = uf(a);

    return {x, u};
}

double f_rhs(double x) {
    // Choose f so exact solution is u=sin(pi x) on [0,1] with u(0)=u(1)=0:
    // -u'' = pi^2 sin(pi x)
    return (M_PI * M_PI) * std::sin(M_PI * x);
}

int main() {
    // ---- Heat FD (theta method) ----
    Grid1D grid(1.0, 101);
    double alpha = 0.05;

    VectorXd u0(grid.N);
    for (int i = 0; i < grid.N; ++i) u0(i) = std::sin(M_PI * grid.x[i]);
    u0(0) = 0.0; u0(grid.N - 1) = 0.0;

    double dt = 0.4 * (grid.dx * grid.dx) / alpha; // stable for explicit, also fine for theta
    int steps = 200;

    std::vector<double> t;
    std::vector<VectorXd> u_hist;
    heat_fd_theta(u0, alpha, grid, dt, steps, 0.5, t, u_hist); // Crank–Nicolson

    std::cout << "Heat FD done. Final time = " << t.back() << "\n";

    // Save final profile to CSV
    {
        std::ofstream ofs("heat_fd_final.csv");
        ofs << "x,u\n";
        const VectorXd& uF = u_hist.back();
        for (int i = 0; i < grid.N; ++i) {
            ofs << grid.x[i] << "," << uF(i) << "\n";
        }
    }

    // ---- Poisson FEM (P1) ----
    auto [x, u] = fem_poisson_p1(40, 1.0, f_rhs, 0.0, 0.0);
    double err_inf = 0.0;
    for (int i = 0; i < (int)x.size(); ++i) {
        double uex = std::sin(M_PI * x[i]);
        err_inf = std::max(err_inf, std::abs(u(i) - uex));
    }
    std::cout << "FEM Poisson max error = " << err_inf << "\n";

    // Save FEM solution
    {
        std::ofstream ofs("poisson_fem.csv");
        ofs << "x,u,uex\n";
        for (int i = 0; i < (int)x.size(); ++i) {
            double uex = std::sin(M_PI * x[i]);
            ofs << x[i] << "," << u(i) << "," << uex << "\n";
        }
    }

    std::cout << "Wrote heat_fd_final.csv and poisson_fem.csv\n";
    return 0;
}
