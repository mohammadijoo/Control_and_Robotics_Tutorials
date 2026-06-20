/* Chapter17_Lesson1.cpp
   Random Variables, Random Processes, and Stationarity Concepts
   C++ implementation for System Dynamics (Chapter 17, Lesson 1)
*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

static double mean_of_vector(const std::vector<double>& v) {
    double s = std::accumulate(v.begin(), v.end(), 0.0);
    return s / static_cast<double>(v.size());
}

static double variance_of_vector(const std::vector<double>& v) {
    double m = mean_of_vector(v);
    double s2 = 0.0;
    for (double x : v) {
        double d = x - m;
        s2 += d * d;
    }
    return s2 / static_cast<double>(v.size());
}

int main() {
    std::mt19937 rng(17);
    std::uniform_real_distribution<double> unif(0.0, 2.0 * M_PI);
    std::normal_distribution<double> n_alpha(0.5, 0.2);
    std::normal_distribution<double> n_beta(0.0, 1.0);

    const double A = 2.0;
    const double omega = 3.0;
    const int Nt = 401;
    const int M = 2000;
    const double t0 = 0.0;
    const double t1 = 4.0;
    const double dt = (t1 - t0) / (Nt - 1);

    std::vector<double> t(Nt);
    for (int i = 0; i < Nt; ++i) {
        t[i] = t0 + i * dt;
    }

    // Ensemble means and variances for X(t) = A cos(omega t + Theta)
    std::vector<double> meanX(Nt, 0.0), meanX2(Nt, 0.0);

    for (int m = 0; m < M; ++m) {
        double theta = unif(rng);
        for (int i = 0; i < Nt; ++i) {
            double x = A * std::cos(omega * t[i] + theta);
            meanX[i] += x;
            meanX2[i] += x * x;
        }
    }
    for (int i = 0; i < Nt; ++i) {
        meanX[i] /= static_cast<double>(M);
        meanX2[i] /= static_cast<double>(M);
    }

    std::vector<double> varX(Nt, 0.0);
    for (int i = 0; i < Nt; ++i) {
        varX[i] = meanX2[i] - meanX[i] * meanX[i];
    }

    // Ensemble autocorrelation estimates at selected lags
    std::vector<int> lags = {0, 10, 30, 60};
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Random-phase harmonic process estimates\n";
    for (int L : lags) {
        double sum = 0.0;
        int count = 0;
        for (int m = 0; m < M; ++m) {
            double theta = unif(rng); // independent realizations for correlation estimate
            for (int i = 0; i < Nt - L; ++i) {
                double x1 = A * std::cos(omega * t[i] + theta);
                double x2 = A * std::cos(omega * t[i + L] + theta);
                sum += x1 * x2;
                ++count;
            }
        }
        double Rhat = sum / static_cast<double>(count);
        double tau = L * dt;
        double Rth = 0.5 * A * A * std::cos(omega * tau);
        std::cout << "lag=" << std::setw(3) << L
                  << ", tau=" << tau
                  << ", R_hat=" << Rhat
                  << ", R_theory=" << Rth << "\n";
    }

    std::cout << "\nSelected mean/variance samples for X(t):\n";
    for (int idx : {0, 100, 200, 400}) {
        std::cout << "t=" << t[idx]
                  << ", mean=" << meanX[idx]
                  << ", var=" << varX[idx] << "\n";
    }

    // Nonstationary process Y(t) = alpha t + beta
    std::vector<double> meanY(Nt, 0.0), meanY2(Nt, 0.0);
    for (int m = 0; m < M; ++m) {
        double alpha = n_alpha(rng);
        double beta = n_beta(rng);
        for (int i = 0; i < Nt; ++i) {
            double y = alpha * t[i] + beta;
            meanY[i] += y;
            meanY2[i] += y * y;
        }
    }
    for (int i = 0; i < Nt; ++i) {
        meanY[i] /= static_cast<double>(M);
        meanY2[i] /= static_cast<double>(M);
    }

    std::cout << "\nNonstationary affine-in-time process estimates:\n";
    for (int idx : {0, 100, 200, 400}) {
        double varY = meanY2[idx] - meanY[idx] * meanY[idx];
        std::cout << "t=" << t[idx]
                  << ", mean=" << meanY[idx]
                  << ", var=" << varY << "\n";
    }

    // AR(1) process in stationary regime
    const double a = 0.8;
    const double sigma_w = 1.0;
    const int K = 400;
    const int Mar = 500;
    std::normal_distribution<double> n_w(0.0, sigma_w);
    std::normal_distribution<double> n_x0(0.0, sigma_w / std::sqrt(1.0 - a * a));

    std::vector<double> meanAR(K, 0.0), meanAR2(K, 0.0);
    for (int m = 0; m < Mar; ++m) {
        std::vector<double> x(K, 0.0);
        x[0] = n_x0(rng);
        for (int k = 1; k < K; ++k) {
            x[k] = a * x[k - 1] + n_w(rng);
        }
        for (int k = 0; k < K; ++k) {
            meanAR[k] += x[k];
            meanAR2[k] += x[k] * x[k];
        }
    }
    for (int k = 0; k < K; ++k) {
        meanAR[k] /= static_cast<double>(Mar);
        meanAR2[k] /= static_cast<double>(Mar);
    }

    std::cout << "\nAR(1) stationary regime estimates:\n";
    for (int k : {0, 100, 200, 399}) {
        double varAR = meanAR2[k] - meanAR[k] * meanAR[k];
        std::cout << "k=" << k
                  << ", mean=" << meanAR[k]
                  << ", var=" << varAR << "\n";
    }
    std::cout << "Theoretical stationary variance = " << (sigma_w * sigma_w) / (1.0 - a * a) << "\n";

    return 0;
}
