#include <iostream>
#include <cmath>

int main() {
    const double K = 1.0;
    const double tau = 0.4;
    const double t_end = 5.0 * tau;
    const int N = 200;
    const double dt = t_end / static_cast<double>(N - 1);

    for (int k = 0; k != N; ++k) {
        double t = k * dt;
        double y = K * (1.0 - std::exp(-t / tau));
        std::cout << t << "," << y << std::endl;
    }
    return 0;
}
