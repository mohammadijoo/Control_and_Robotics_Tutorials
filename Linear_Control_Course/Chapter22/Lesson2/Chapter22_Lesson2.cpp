#include <iostream>
#include <complex>
#include <cmath>

int main() {
    using std::complex;
    using std::cout;
    using std::endl;

    const double Kp = 2.0;
    const double tau_p = 0.1;
    const double Kc = 5.0;

    // Loop over frequencies (rad/s)
    for (double w = 1.0; w <= 1000.0; w *= 10.0) {
        complex<double> s(0.0, w);  // s = j w
        complex<double> P = Kp / (tau_p * s + 1.0);
        complex<double> C = Kc;
        complex<double> L = C * P;
        complex<double> T = L / (1.0 + L);

        double magT = std::abs(T);
        cout << "w = " << w
             << ", |T(jw)| = " << magT << endl;
    }
    return 0;
}
