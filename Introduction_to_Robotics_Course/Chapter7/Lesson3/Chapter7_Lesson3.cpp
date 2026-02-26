#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

int main() {
    const double fs = 200000.0;
    const double f0 = 40000.0;
    const double v_air = 343.0;
    const int N = 2000;         // 0.01 s at 200 kHz
    const int pulseN = 200;     // 0.001 s pulse

    std::vector<double> s(N, 0.0), r(N, 0.0);

    // transmit pulse
    for(int n = 0; n < pulseN; ++n) {
        double t = n / fs;
        s[n] = std::sin(2*M_PI*f0*t);
    }

    // create delayed echo
    double T_true = 0.0075;
    int dSamp = static_cast<int>(T_true * fs);
    double alpha = 0.6;
    for(int n = dSamp; n < N; ++n) {
        r[n] = alpha * s[n - dSamp];
    }

    // full correlation C[k] = sum_n r[n] s[n-k]
    std::vector<double> C(2*N - 1, 0.0);
    for(int k = -(N-1); k <= (N-1); ++k) {
        double sum = 0.0;
        for(int n = 0; n < N; ++n) {
            int idx = n - k;
            if(idx >= 0 && idx < N) sum += r[n] * s[idx];
        }
        C[k + (N-1)] = sum;
    }

    auto it = std::max_element(C.begin(), C.end());
    int k_hat = static_cast<int>(std::distance(C.begin(), it)) - (N-1);
    double T_hat = k_hat / fs;
    double d_hat = v_air * T_hat / 2.0;

    std::cout << "Estimated delay: " << T_hat << " s\n";
    std::cout << "Estimated range: " << d_hat << " m\n";
    return 0;
}
