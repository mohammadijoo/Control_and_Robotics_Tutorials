#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

using std::complex;
using std::vector;
using std::cout;
using std::endl;

using dcomplex = complex<double>;

// Evaluate polynomial p(s) = c[0] + c[1] s + ... + c[N] s^N at s
dcomplex poly_eval(const vector<double>& c, dcomplex s) {
    dcomplex val(0.0, 0.0);
    dcomplex p(1.0, 0.0);
    for (double coeff : c) {
        val += coeff * p;
        p *= s;
    }
    return val;
}

int main() {
    // Example: L(s) = K / (s (T s + 1)) = K / (T s^2 + s)
    double K = 10.0;
    double T = 0.1;

    vector<double> num{K};           // b0
    vector<double> den{0.0, 1.0, T}; // a0 + a1 s + a2 s^2 = T s^2 + s

    // Logarithmic frequency grid
    int N = 2000;
    double w_min = 0.1;
    double w_max = 1000.0;
    vector<double> w(N);
    for (int i = 0; i < N; ++i) {
        double alpha = static_cast<double>(i) / (N - 1);
        w[i] = w_min * std::pow(w_max / w_min, alpha);
    }

    auto sign = [](double x) {
        if (x > 0.0) return 1;
        if (x < 0.0) return -1;
        return 0;
    };

    double w_gc = -1.0;
    double w_pc = -1.0;

    double prev_g = 0.0;
    double prev_h = 0.0;
    int prev_g_sign = 0;
    int prev_h_sign = 0;
    bool first = true;

    for (int i = 0; i < N; ++i) {
        dcomplex s(0.0, w[i]);
        dcomplex L = poly_eval(num, s) / poly_eval(den, s);
        double mag = std::abs(L);
        double phase = std::arg(L); // radians, in (-pi, pi]

        double g = mag - 1.0;
        double phase_wrapped = std::atan2(std::sin(phase), std::cos(phase));
        double h = phase_wrapped + M_PI; // zero at -pi

        int sg = sign(g);
        int sh = sign(h);

        if (!first) {
            if (w_gc < 0.0 && sg != 0 && sg != prev_g_sign && prev_g_sign != 0) {
                double t = prev_g / (prev_g - g);
                w_gc = w[i - 1] + t * (w[i] - w[i - 1]);
            }
            if (w_pc < 0.0 && sh != 0 && sh != prev_h_sign && prev_h_sign != 0) {
                double t = prev_h / (prev_h - h);
                w_pc = w[i - 1] + t * (w[i] - w[i - 1]);
            }
        } else {
            first = false;
        }

        prev_g = g;
        prev_h = h;
        prev_g_sign = sg;
        prev_h_sign = sh;
    }

    cout << "Approx gain crossover w_gc = " << w_gc << " rad/s" << endl;
    cout << "Approx phase crossover w_pc = " << w_pc << " rad/s" << endl;

    return 0;
}
