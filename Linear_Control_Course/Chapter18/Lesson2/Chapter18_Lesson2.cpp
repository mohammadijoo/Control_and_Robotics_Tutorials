#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

int main() {
    using std::complex;
    using std::cout;
    using std::endl;

    const double J = 0.01;
    const double b = 0.1;
    const double Kp = 5.0;
    const double Ki = 10.0;

    // Frequency grid (rad/s)
    std::vector<double> w;
    for (int i = 0; i <= 20; ++i) {
        double wi = std::pow(10.0, -1.0 + 0.15 * i); // 0.1 ... about 31.6
        w.push_back(wi);
    }

    cout << "# w, |Gd(jw)|" << endl;
    for (double wi : w) {
        complex<double> j(0.0, 1.0);
        complex<double> s = j * wi;

        // P(s) = 1 / (J s^2 + b s)
        complex<double> denomP = J * s * s + b * s;
        complex<double> P = 1.0 / denomP;

        // C(s) = Kp + Ki/s
        complex<double> C = Kp + Ki / s;

        complex<double> L = C * P;
        complex<double> Gd = P / (complex<double>(1.0, 0.0) + L);

        double mag = std::abs(Gd);
        cout << wi << " " << mag << endl;
    }

    return 0;
}
