#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

int main() {
    using std::complex;
    using std::vector;

    const double J = 0.01;
    const double B = 0.1;
    const double Kp = 50.0;
    const double Kd = 2.0;

    // Frequency grid (rad/s)
    vector<double> omega;
    for (int k = 0; k <= 400; ++k) {
        double logw = -1.0 + 4.0 * (static_cast<double>(k) / 400.0); // from 10^-1 to 10^3
        omega.push_back(std::pow(10.0, logw));
    }

    for (double w : omega) {
        complex<double> jw(0.0, w);

        // Plant P(jw) = 1 / (J (jw)^2 + B jw)
        complex<double> denomP = J * jw * jw + B * jw;
        complex<double> P = 1.0 / denomP;

        // PD controller C(jw) = Kp + Kd jw
        complex<double> C = Kp + Kd * jw;

        complex<double> L = C * P;
        complex<double> S = 1.0 / (1.0 + L);
        complex<double> T = L / (1.0 + L);

        double magS = std::abs(S);
        double magT = std::abs(T);

        // Example: print a few points around 10 rad/s
        if (w > 9.5 && w < 10.5) {
            std::cout << "w = " << w
                      << " |S(jw)| = " << magS
                      << " |T(jw)| = " << magT
                      << std::endl;
        }
    }

    return 0;
}
