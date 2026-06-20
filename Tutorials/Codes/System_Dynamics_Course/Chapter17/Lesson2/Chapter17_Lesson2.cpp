// Chapter17_Lesson2.cpp
// Autocorrelation and PSD (compact C++17 example)

#include <cmath>
#include <complex>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

std::vector<double> autocorrBiased(const std::vector<double>& x, int M) {
    int N = (int)x.size();
    double mu = std::accumulate(x.begin(), x.end(), 0.0) / N;
    std::vector<double> xc(N), R(M + 1, 0.0);
    for (int i = 0; i < N; ++i) xc[i] = x[i] - mu;
    for (int m = 0; m <= M; ++m) {
        for (int n = 0; n < N - m; ++n) R[m] += xc[n] * xc[n + m];
        R[m] /= N;
    }
    return R;
}

std::vector<double> evenExtend(const std::vector<double>& R) {
    int M = (int)R.size() - 1;
    std::vector<double> e;
    for (int i = 0; i <= M; ++i) e.push_back(R[i]);
    for (int i = M - 1; i >= 1; --i) e.push_back(R[i]);
    return e;
}

std::vector<double> dftReal(const std::vector<double>& x) {
    int N = (int)x.size();
    std::vector<double> S(N, 0.0);
    for (int k = 0; k < N; ++k) {
        std::complex<double> Xk(0.0, 0.0);
        for (int n = 0; n < N; ++n) {
            double a = -2.0 * M_PI * k * n / N;
            Xk += x[n] * std::complex<double>(std::cos(a), std::sin(a));
        }
        S[k] = Xk.real(); // real because autocorrelation extension is even
    }
    return S;
}

int main() {
    const int N = 512, M = 20;
    const double sigma = 1.5, b = 0.6;
    std::mt19937 gen(17);
    std::normal_distribution<double> gauss(0.0, sigma);

    std::vector<double> w(N), x(N);
    for (int n = 0; n < N; ++n) w[n] = gauss(gen);
    x[0] = w[0];
    for (int n = 1; n < N; ++n) x[n] = w[n] + b * w[n - 1];

    auto Rw = autocorrBiased(w, M);
    auto Rx = autocorrBiased(x, M);
    auto Sx = dftReal(evenExtend(Rx));

    std::cout << "R_w[0] = " << Rw[0] << "\n";
    std::cout << "R_x[0] = " << Rx[0] << "\n";
    std::cout << "Theo R_x[0] = " << sigma * sigma * (1.0 + b * b) << "\n";
    std::cout << "First 8 PSD bins from autocorrelation-DFT:\n";
    for (int k = 0; k < 8; ++k) std::cout << "k=" << k << "  " << Sx[k] << "\n";
    return 0;
}
