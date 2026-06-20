// Chapter17_Lesson3.cpp
// Response of Linear Systems to Random Inputs: Mean and Variance Propagation
// C++ implementation: discrete-time scalar and 2-state examples (Monte Carlo + theory)

#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include <cmath>

int main() {
    std::cout << std::fixed << std::setprecision(6);

    // Example 1: Scalar discrete-time system
    // x_{k+1} = a x_k + b mu_u + g w_k,  w_k ~ N(0, q)
    const double a = 0.95;
    const double b = 0.40;
    const double g = 1.0;
    const double mu_u = 1.2;
    const double q = 0.30;
    const int K = 120;
    const int M = 20000;

    // Theoretical mean and variance propagation
    double m = 0.0;
    double P = 0.0;
    for (int k = 0; k < K; ++k) {
        m = a * m + b * mu_u;
        P = a * a * P + g * g * q;
    }

    // Monte Carlo verification
    std::mt19937 rng(42);
    std::normal_distribution<double> N01(0.0, 1.0);

    std::vector<double> x(M, 0.0);
    for (int k = 0; k < K; ++k) {
        for (int i = 0; i < M; ++i) {
            double wk = std::sqrt(q) * N01(rng);
            x[i] = a * x[i] + b * mu_u + g * wk;
        }
    }

    double mean_mc = 0.0;
    for (double xi : x) mean_mc += xi;
    mean_mc /= M;

    double var_mc = 0.0;
    for (double xi : x) {
        double d = xi - mean_mc;
        var_mc += d * d;
    }
    var_mc /= (M - 1);

    double P_inf = (g * g * q) / (1.0 - a * a);
    double m_inf = (b * mu_u) / (1.0 - a);

    std::cout << "Scalar example (after " << K << " steps)\n";
    std::cout << "Theory mean      = " << m << "\n";
    std::cout << "Monte Carlo mean = " << mean_mc << "\n";
    std::cout << "Theory variance  = " << P << "\n";
    std::cout << "Monte Carlo var  = " << var_mc << "\n";
    std::cout << "Steady mean      = " << m_inf << "\n";
    std::cout << "Steady variance  = " << P_inf << "\n\n";

    // Example 2: 2-state stable linear system with white process noise
    // x_{k+1} = A x_k + G w_k, w_k ~ N(0, Q), mean zero input
    double A[2][2] = {{0.90, 0.10},
                      {-0.20, 0.80}};
    double G[2][1] = {{0.0},
                      {1.0}};
    double Q = 0.20;

    // Propagate covariance P_{k+1} = A P A^T + G Q G^T
    double Pm[2][2] = {{0.0, 0.0},
                       {0.0, 0.0}};

    for (int k = 0; k < 200; ++k) {
        // T = A P
        double T[2][2];
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                T[i][j] = A[i][0] * Pm[0][j] + A[i][1] * Pm[1][j];
            }
        }

        // APAT = T A^T
        double APAT[2][2];
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                APAT[i][j] = T[i][0] * A[j][0] + T[i][1] * A[j][1];
            }
        }

        // G Q G^T
        double GQGT[2][2] = {
            {G[0][0] * Q * G[0][0], G[0][0] * Q * G[1][0]},
            {G[1][0] * Q * G[0][0], G[1][0] * Q * G[1][0]}
        };

        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                Pm[i][j] = APAT[i][j] + GQGT[i][j];
    }

    std::cout << "2-state covariance after 200 steps (theory recursion):\n";
    std::cout << "[" << Pm[0][0] << ", " << Pm[0][1] << "]\n";
    std::cout << "[" << Pm[1][0] << ", " << Pm[1][1] << "]\n";

    return 0;
}
