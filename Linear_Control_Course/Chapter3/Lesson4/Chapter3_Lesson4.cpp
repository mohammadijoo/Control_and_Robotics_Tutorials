#include <iostream>
#include <vector>

int main() {
    double R = 10e3;      // 10 kOhm
    double C = 1e-6;      // 1 microfarad
    double RC = R * C;

    double h = 1e-4;      // 0.1 ms step
    double T = 0.1;       // 0.1 s total
    int N = static_cast<int>(T / h);

    double vC = 0.0;      // initial capacitor voltage
    double U0 = 1.0;      // step input

    std::vector<double> time;
    std::vector<double> vC_hist;
    time.reserve(N + 1);
    vC_hist.reserve(N + 1);

    time.push_back(0.0);
    vC_hist.push_back(vC);

    for (int k = 0; k < N; ++k) {
        double t = (k + 1) * h;
        double u = U0; // step

        double dvC = (u - vC) / RC;
        vC += h * dvC;

        time.push_back(t);
        vC_hist.push_back(vC);
    }

    // Print a few samples for inspection
    for (int k = 0; k <= N; k += N / 10) {
        std::cout << "t = " << time[k]
                  << " s, vC = " << vC_hist[k] << " V\n";
    }

    return 0;
}
