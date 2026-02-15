
#include <iostream>
#include <vector>
#include <cmath>

int main() {
    const double m = 1.0;
    const double b = 1.0;
    const double k_p = 200.0;
    const double k_d = 20.0;
    const double k_e = 5000.0;
    const double x_s = 0.0;
    const double F_max = 80.0;
    const double dt = 0.0005;
    const double T = 1.0;
    const int N = static_cast<int>(T / dt);

    auto x_d = [](double t) {
        return (t < 0.1) ? -0.02 : 0.03;
    };

    double x = -0.02;
    double v = 0.0;

    std::vector<double> xs(N), Fs(N), us(N);

    for (int i = 0; i < N; ++i) {
        double t = i * dt;

        double F_e = (x <= x_s) ? 0.0 : k_e * (x - x_s);

        double e = x_d(t) - x;
        double u = k_p * e - k_d * v;

        if (F_e > F_max) {
            u -= (F_e - F_max);
        }

        double a = (u - F_e - b * v) / m;
        v += dt * a;
        x += dt * v;

        xs[i] = x;
        Fs[i] = F_e;
        us[i] = u;
    }

    // Print final values as a sanity check
    std::cout << "Final x = " << xs.back()
              << ", Final F_e = " << Fs.back() << std::endl;
    return 0;
}
