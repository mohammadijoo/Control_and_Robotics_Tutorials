
#include <iostream>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

int main() {
    const int n = 2;
    const double Ts = 0.02;
    const int N = 20;

    const int nx = 2 * n;
    const int nu = n;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd A(nx, nx);
    Eigen::MatrixXd B(nx, nu);

    A.setZero();
    A.block(0, 0, n, n) = I;
    A.block(0, n, n, n) = Ts * I;
    A.block(n, n, n, n) = I;

    B.setZero();
    B.block(n, 0, n, n) = Ts * I;

    // Weights
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx, nx);
    Q.diagonal() << 100.0, 100.0, 10.0, 10.0;
    Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(nu, nu);

    // Build Phi, Gamma and H, similar to the Python code (omitted for brevity).
    // Suppose H is (N * nu) x (N * nu) and q is length N * nu.
    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd q;

    // Box constraints on u
    Eigen::VectorXd u_min = -5.0 * Eigen::VectorXd::Ones(nu);
    Eigen::VectorXd u_max =  5.0 * Eigen::VectorXd::Ones(nu);

    Eigen::SparseMatrix<double> Aineq(2 * N * nu, N * nu);
    Eigen::VectorXd l(2 * N * nu), u(2 * N * nu);

    // Aineq = [I; -I]
    Aineq.setIdentity();
    for (int i = 0; i < N * nu; ++i) {
        Aineq.insert(N * nu + i, i) = -1.0;
    }

    for (int k = 0; k < N; ++k) {
        for (int j = 0; j < nu; ++j) {
            int idx = k * nu + j;
            l(idx) = -OsqpEigen::INFTY;
            u(idx) = u_max(j);
            l(N * nu + idx) = -OsqpEigen::INFTY;
            u(N * nu + idx) = -u_min(j);
        }
    }

    OsqpEigen::Solver solver;
    solver.settings().setWarmStart(true);
    solver.data().setNumberOfVariables(N * nu);
    solver.data().setNumberOfConstraints(2 * N * nu);
    if (!solver.data().setHessianMatrix(H)) return 1;
    if (!solver.data().setGradient(q)) return 1;
    if (!solver.data().setLinearConstraintsMatrix(Aineq)) return 1;
    if (!solver.data().setLowerBound(l)) return 1;
    if (!solver.data().setUpperBound(u)) return 1;

    if (!solver.initSolver()) return 1;

    // At runtime, update q when x0 changes, then solve:
    // solver.updateGradient(q_new);
    // solver.solveProblem();
    // Eigen::VectorXd U_opt = solver.getSolution();
    // Eigen::VectorXd u0 = U_opt.segment(0, nu);

    std::cout << "C++ MPC setup complete." << std::endl;
    return 0;
}
