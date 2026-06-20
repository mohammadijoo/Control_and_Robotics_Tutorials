/*
Chapter 13 - Vibrations and Multi-Degree-of-Freedom (MDOF) Systems
Lesson 5 - Intro to Experimental Modal Analysis and System Identification Concepts

File: Chapter13_Lesson5.cpp

This is a compact, didactic C++ example that:
1) Simulates a 3-DOF state-space model with RK4 integration.
2) Estimates an FRF using the H1 estimator via a simple DFT (O(N^2)).

Notes:
- For production, replace the DFT with FFT (e.g., FFTW) and use robust windowing/averaging.
- Requires Eigen (header-only): https://eigen.tuxfamily.org
Build (example):
  g++ -O2 -std=c++17 Chapter13_Lesson5.cpp -I path/to/eigen -o Chapter13_Lesson5
*/

#include <Eigen/Dense>
#include <complex>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>

using Eigen::MatrixXd;
using Eigen::VectorXd;

static constexpr double PI = 3.14159265358979323846;

struct Spectra {
    std::vector<double> f;
    std::vector<std::complex<double>> Guu;
    std::vector<std::complex<double>> Gyy;
    std::vector<std::complex<double>> Gyu;
};

std::vector<std::complex<double>> dft(const std::vector<double>& x) {
    const int N = (int)x.size();
    std::vector<std::complex<double>> X(N);
    for (int k = 0; k < N; ++k) {
        std::complex<double> s(0.0, 0.0);
        for (int n = 0; n < N; ++n) {
            double ang = -2.0 * PI * k * n / (double)N;
            s += std::complex<double>(std::cos(ang), std::sin(ang)) * x[n];
        }
        X[k] = s;
    }
    return X;
}

Spectra spectra_single_segment(const std::vector<double>& u, const std::vector<double>& y, double fs) {
    // Single-segment (no averaging) autospectra and cross-spectrum using DFT:
    // Guu(k) = (1/N) U(k) conj(U(k)), etc.
    const int N = (int)u.size();
    auto U = dft(u);
    auto Y = dft(y);

    Spectra out;
    const int K = N/2 + 1;
    out.f.resize(K);
    out.Guu.resize(K);
    out.Gyy.resize(K);
    out.Gyu.resize(K);

    for (int k = 0; k < K; ++k) {
        out.f[k] = (fs * k) / (double)N;
        std::complex<double> Uc = std::conj(U[k]);
        std::complex<double> Yc = std::conj(Y[k]);
        out.Guu[k] = (U[k] * Uc) / (double)N;
        out.Gyy[k] = (Y[k] * Yc) / (double)N;
        out.Gyu[k] = (Y[k] * Uc) / (double)N;
    }
    return out;
}

int main() {
    // 3-DOF chain example
    MatrixXd M = MatrixXd::Zero(3,3);
    M(0,0) = 1.0; M(1,1) = 0.9; M(2,2) = 0.8;

    double k1=800, k2=600, k3=500, k4=700;
    MatrixXd K(3,3);
    K << k1+k2, -k2,    0,
         -k2,   k2+k3, -k3,
          0,    -k3,   k3+k4;

    double c1=2.0, c2=1.8, c3=1.5, c4=2.2;
    MatrixXd C(3,3);
    C << c1+c2, -c2,    0,
         -c2,   c2+c3, -c3,
          0,    -c3,   c3+c4;

    // State-space: x=[q; qd], xdot = A x + B f
    MatrixXd Minv = M.inverse();
    MatrixXd Z = MatrixXd::Zero(3,3);
    MatrixXd I = MatrixXd::Identity(3,3);

    MatrixXd A(6,6);
    A << Z, I,
         -Minv*K, -Minv*C;

    MatrixXd B = MatrixXd::Zero(6,3);
    B.block(3,0,3,3) = Minv;

    // Acceleration output: qdd = -Minv*K q - Minv*C qd + Minv f
    MatrixXd Ca(3,6);
    Ca << -Minv*K, -Minv*C;

    const double fs = 500.0;
    const double dt = 1.0 / fs;
    const double T = 10.0;
    const int N = (int)std::floor(T * fs);

    // Input: band-limited noise (very simple smoothing)
    std::mt19937 gen(7);
    std::normal_distribution<double> nd(0.0, 1.0);

    std::vector<double> u(N, 0.0);
    for (int i = 0; i < N; ++i) u[i] = nd(gen);

    // crude lowpass: moving average
    const int W = 9;
    std::vector<double> u_lp(N, 0.0);
    for (int i = 0; i < N; ++i) {
        double s = 0.0;
        int cnt = 0;
        for (int j = -W; j <= W; ++j) {
            int k = i + j;
            if (k >= 0 && k < N) { s += u[k]; cnt++; }
        }
        u_lp[i] = s / (double)cnt;
    }
    u = u_lp;

    // simulate (force at DOF0, measure accel at DOF0)
    VectorXd x = VectorXd::Zero(6);
    std::vector<double> y(N, 0.0);

    auto fvec = [&](int i)->VectorXd{
        VectorXd f = VectorXd::Zero(3);
        f(0) = u[i];
        return f;
    };

    auto rhs = [&](const VectorXd& xk, const VectorXd& fk)->VectorXd{
        return A * xk + B * fk;
    };

    for (int i = 0; i < N; ++i) {
        VectorXd f = fvec(i);

        // RK4
        VectorXd k1 = rhs(x, f);
        VectorXd k2 = rhs(x + 0.5*dt*k1, f);
        VectorXd k3 = rhs(x + 0.5*dt*k2, f);
        VectorXd k4 = rhs(x + dt*k3, f);
        x = x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);

        // acceleration output (without direct term Minv*f for simplicity; add it)
        VectorXd acc = Ca * x + Minv * f;
        y[i] = acc(0);
    }

    // FRF estimate with H1 (single-segment)
    Spectra sp = spectra_single_segment(u, y, fs);
    const int Kbins = (int)sp.f.size();

    std::cout << "f_Hz, |H1|, coherence\n";
    for (int k = 1; k < Kbins && sp.f[k] <= 120.0; ++k) {
        std::complex<double> H1 = sp.Gyu[k] / (sp.Guu[k] + std::complex<double>(1e-30,0.0));
        double coh = std::norm(sp.Gyu[k]) / ( (std::real(sp.Guu[k])*std::real(sp.Gyy[k]) + 1e-30) );
        std::cout << sp.f[k] << ", " << std::abs(H1) << ", " << coh << "\n";
    }

    std::cout << "\nDone.\n";
    return 0;
}
