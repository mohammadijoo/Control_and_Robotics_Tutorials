#include <cmath>
#include <iostream>

double zeta_from_Mp(double Mp_percent) {
    double Mp = Mp_percent / 100.0;
    if (Mp <= 0.0 || Mp >= 1.0) {
        throw std::runtime_error("Mp_percent must be between 0 and 100.");
    }
    double lnMp = std::log(Mp);
    double zeta = -lnMp / std::sqrt(M_PI * M_PI + lnMp * lnMp);
    return zeta;
}

double sigma_from_Ts(double Ts, double perc = 2.0) {
    if (Ts <= 0.0) throw std::runtime_error("Ts must be positive.");
    if (std::fabs(perc - 2.0) < 1e-6) {
        return -4.0 / Ts;
    } else if (std::fabs(perc - 5.0) < 1e-6) {
        return -3.0 / Ts;
    } else {
        double eps = perc / 100.0;
        return std::log(eps) / Ts; // negative
    }
}

bool check_pole_constraints(double sigma, double omega_d,
                            double Mp_percent, double Ts) {
    if (sigma >= 0.0) return false; // unstable

    double omega_n = std::sqrt(sigma * sigma + omega_d * omega_d);
    double zeta = -sigma / omega_n;

    double zeta_min = zeta_from_Mp(Mp_percent);
    double sigma_max = sigma_from_Ts(Ts, 2.0);

    return (zeta >= zeta_min) && (sigma <= sigma_max);
}

int main() {
    double sigma = -2.0;
    double omega_d = 3.0;
    double Mp_spec = 10.0;
    double Ts_spec = 2.0;

    bool ok = check_pole_constraints(sigma, omega_d, Mp_spec, Ts_spec);
    std::cout << "Pole s = " << sigma << " + j" << omega_d
              << " satisfies specs? " << (ok ? "yes" : "no")
              << std::endl;
    return 0;
}
