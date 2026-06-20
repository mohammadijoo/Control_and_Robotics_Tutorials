\
/* Chapter16_Lesson1.cpp
   Sampling, aliasing, and zero-order hold data generation.
   Compile (example): g++ -std=c++17 Chapter16_Lesson1.cpp -O2 -o Chapter16_Lesson1
*/

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

double aliasFrequency(double f0, double fs) {
    long m = static_cast<long>(std::llround(f0 / fs));
    double fa = std::fabs(f0 - m * fs);
    if (fa > fs / 2.0) {
        fa = fs - fa;
    }
    return fa;
}

int main() {
    const double fs = 80.0;
    const double Ts = 1.0 / fs;
    const double f1 = 12.0;
    const double f2 = 55.0;
    const double A1 = 1.0;
    const double A2 = 0.7;
    const double duration = 0.25;

    const double fdense = 5000.0;
    const double dt = 1.0 / fdense;
    const int Nd = static_cast<int>(duration * fdense);
    const int Ns = static_cast<int>(duration * fs);

    std::vector<double> td(Nd), xCont(Nd), xZoh(Nd);
    std::vector<double> ts(Ns), xSamp(Ns);

    // Sampled sequence
    for (int k = 0; k < Ns; ++k) {
        ts[k] = k * Ts;
        xSamp[k] = A1 * std::sin(2.0 * M_PI * f1 * ts[k]) +
                   A2 * std::sin(2.0 * M_PI * f2 * ts[k]);
    }

    // Dense reference and ZOH
    for (int i = 0; i < Nd; ++i) {
        td[i] = i * dt;
        xCont[i] = A1 * std::sin(2.0 * M_PI * f1 * td[i]) +
                   A2 * std::sin(2.0 * M_PI * f2 * td[i]);

        int k = static_cast<int>(std::floor(td[i] / Ts));
        if (k < 0) k = 0;
        if (k >= Ns) k = Ns - 1;
        xZoh[i] = xSamp[k];
    }

    // Export CSV for plotting in any tool
    std::ofstream file("Chapter16_Lesson1_cpp_output.csv");
    file << "t_dense,x_cont,x_zoh\n";
    file << std::fixed << std::setprecision(8);
    for (int i = 0; i < Nd; ++i) {
        file << td[i] << "," << xCont[i] << "," << xZoh[i] << "\n";
    }
    file << "\n";
    file << "k,ts,x_sample\n";
    for (int k = 0; k < Ns; ++k) {
        file << k << "," << ts[k] << "," << xSamp[k] << "\n";
    }
    file.close();

    std::cout << "Sampling frequency fs = " << fs << " Hz\n";
    std::cout << "Nyquist frequency = " << fs / 2.0 << " Hz\n";
    std::cout << "Input component f2 = " << f2 << " Hz aliases to "
              << aliasFrequency(f2, fs) << " Hz\n";
    std::cout << "CSV written to Chapter16_Lesson1_cpp_output.csv\n";

    return 0;
}
