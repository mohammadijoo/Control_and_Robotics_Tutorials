#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

using cd = std::complex<double>;

// Evaluate P(jw) = 1 / (jw^2 + 2*zeta*wn*jw + wn^2)
cd Pjw(double w, double zeta, double wn) {
    cd jw(0.0, w);
    cd denom = jw * jw + cd(2.0 * zeta * wn, 0.0) * jw + cd(wn * wn, 0.0);
    return cd(1.0, 0.0) / denom;
}

// Controller C(s) = K (s + z) / s evaluated at s = jw
cd Cjw(double w, double K, double z) {
    cd jw(0.0, w);
    return K * (jw + cd(z, 0.0)) / jw;
}

// First-order performance weight W_P(s) = (s/M + wp) / (s + wp*A)
cd WPjw(double w, double M, double A, double wp) {
    cd jw(0.0, w);
    return (jw / M + cd(wp, 0.0)) / (jw + cd(wp * A, 0.0));
}

// Multiplicative uncertainty weight W_D(s) = (s/alpha + wD) / (s + wD)
cd WDjw(double w, double alpha, double wD) {
    cd jw(0.0, w);
    return (jw / alpha + cd(wD, 0.0)) / (jw + cd(wD, 0.0));
}

int main() {
    double zeta = 0.7, wn = 10.0;
    double K = 20.0, z = 2.0;
    double M_P = 1.5, A_P = 1e-3, wP = 1.0;
    double alpha = 1.5, wD = 30.0;

    std::vector<double> wgrid;
    for (int i = 0; i <= 500; ++i) {
        double frac = static_cast<double>(i) / 500.0;
        double w = std::pow(10.0, -2.0 + 5.0 * frac); // 1e-2 to 1e3
        wgrid.push_back(w);
    }

    double max_WP_S = 0.0;
    double max_WD_T = 0.0;

    for (double w : wgrid) {
        cd P = Pjw(w, zeta, wn);
        cd C = Cjw(w, K, z);
        cd L = C * P;
        cd S = cd(1.0, 0.0) / (cd(1.0, 0.0) + L);
        cd T = L / (cd(1.0, 0.0) + L);

        cd WP = WPjw(w, M_P, A_P, wP);
        cd WD = WDjw(w, alpha, wD);

        double WP_S = std::abs(WP * S);
        double WD_T = std::abs(WD * T);

        if (WP_S > max_WP_S) max_WP_S = WP_S;
        if (WD_T > max_WD_T) max_WD_T = WD_T;
    }

    std::cout << "max |W_P(jw) S(jw)| = " << max_WP_S << std::endl;
    std::cout << "max |W_D(jw) T(jw)| = " << max_WD_T << std::endl;
    return 0;
}
