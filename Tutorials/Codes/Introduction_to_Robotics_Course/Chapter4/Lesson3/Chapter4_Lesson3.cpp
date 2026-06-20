
#include <iostream>
#include <vector>
#include <cmath>

int main() {
    // Power budget
    double Vb = 14.8, C_Ah = 5.0, eta = 0.9;
    std::vector<std::pair<double,double>> loads = {
        {14.8, 2.0}, {5.0, 1.2}, {3.3, 0.5}
    };
    double P_tot = 0.0;
    for (auto &li : loads) P_tot += li.first * li.second;

    double E_b = 3600.0 * Vb * C_Ah;                // Joules
    double T_run_h = eta * E_b / P_tot / 3600.0;    // hours

    std::cout << "Average power [W] = " << P_tot << "\n";
    std::cout << "Estimated runtime [h] = " << T_run_h << "\n";

    // Wire drop
    double rho = 1.68e-8, L = 1.5, A = 1.0e-6, I = 5.0;
    double Rw = rho * L / A;
    double dV = I * Rw;
    std::cout << "Wire resistance [ohm] = " << Rw << "\n";
    std::cout << "Voltage drop [V] = " << dV << "\n";

    // RC cutoff
    double R = 1e3, C = 0.1e-6;
    double fc = 1.0 / (2.0 * M_PI * R * C);
    std::cout << "Cutoff frequency [Hz] = " << fc << "\n";
    return 0;
}
      