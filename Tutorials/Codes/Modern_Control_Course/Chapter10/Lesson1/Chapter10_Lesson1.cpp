/*
Chapter10_Lesson1.cpp

Compile example:
    g++ -std=c++17 Chapter10_Lesson1.cpp -I /path/to/eigen -O2 -o steer

This program demonstrates finite-horizon state steering for the double integrator
using Eigen. The input sequence is piecewise constant.
*/

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>

using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Matrix2d matrixPower(Matrix2d M, int p) {
    Matrix2d result = Matrix2d::Identity();
    for (int i = 0; i < p; ++i) {
        result = result * M;
    }
    return result;
}

int main() {
    const double T = 2.0;
    const int N = 60;
    const double dt = T / static_cast<double>(N);

    Matrix2d Phi;
    Phi << 1.0, dt,
           0.0, 1.0;

    Vector2d Gamma;
    Gamma << 0.5 * dt * dt, dt;

    Vector2d x0(0.0, 0.0);
    Vector2d xf(1.0, 0.0);

    MatrixXd S(2, N);
    for (int k = 0; k < N; ++k) {
        S.col(k) = matrixPower(Phi, N - 1 - k) * Gamma;
    }

    Eigen::FullPivLU<MatrixXd> lu(S);
    int rankS = lu.rank();

    if (rankS < 2) {
        throw std::runtime_error("The finite-horizon steering map is rank deficient.");
    }

    Vector2d freeResponse = matrixPower(Phi, N) * x0;
    Vector2d targetShift = xf - freeResponse;

    Matrix2d gram = S * S.transpose();
    VectorXd u = S.transpose() * gram.ldlt().solve(targetShift);

    MatrixXd x(2, N + 1);
    x.col(0) = x0;

    for (int k = 0; k < N; ++k) {
        x.col(k + 1) = Phi * x.col(k) + Gamma * u(k);
    }

    double energy = dt * u.squaredNorm();

    std::cout << "Rank of finite-horizon steering map S: " << rankS << "\n";
    std::cout << "Requested final state: " << xf.transpose() << "\n";
    std::cout << "Achieved final state: " << x.col(N).transpose() << "\n";
    std::cout << "Final steering error norm: " << (x.col(N) - xf).norm() << "\n";
    std::cout << "Input energy approximation: " << energy << "\n";

    return 0;
}
