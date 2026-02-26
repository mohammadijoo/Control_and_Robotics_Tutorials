#include <iostream>
#include <complex>
#include <vector>

// Example: G(s) = 1 / (s (s + 1)), C(s) = Kp + Ki / s
// Evaluate S(jw) and T(jw) on a frequency grid

int main() {
    using std::complex;
    using std::cout;
    using std::endl;
    const double Kp = 4.0;
    const double Ki = 3.0;
    const complex<double> j(0.0, 1.0);

    auto G = [&] (complex<double> s) {
        return 1.0 / (s * (s + 1.0));
    };
    auto C = [&] (complex<double> s) {
        return Kp + Ki / s;
    };

    std::vector<double> w;
    for (int k = 0; k <= 400; ++k) {
        double wk = std::pow(10.0, -2.0 + 4.0 * k / 400.0);
        w.push_back(wk);
    }

    double Ms = 0.0;
    double Mt = 0.0;
    for (double wk : w) {
        complex<double> s = j * wk;
        complex<double> L = C(s) * G(s);
        complex<double> S = 1.0 / (1.0 + L);
        complex<double> T = L / (1.0 + L);
        double magS = std::abs(S);
        double magT = std::abs(T);
        if (magS > Ms) Ms = magS;
        if (magT > Mt) Mt = magT;
    }
    cout << "M_S = " << Ms << ", M_T = " << Mt << endl;

    // In a ROS controller, the continuous-time design can be discretized and
    // the same logic applied to the discrete-time sensitivity functions.
    return 0;
}
