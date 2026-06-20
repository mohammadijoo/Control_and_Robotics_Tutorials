#include <iostream>
#include <cmath>

double human_command(double time) {
    if (time < 5.0) {
        return 1.0;
    } else {
        return -1.0;
    }
}

int main() {
    const double T = 0.01;
    const double T_end = 10.0;
    const double k_gain = 1.0;
    const double alpha = 0.4;
    const double U_max = 2.0;

    const int N = static_cast<int>(T_end / T);
    double x = 0.0;
    double t = 0.0;

    for (int k = 0; k < N; ++k) {
        double u_h = human_command(t);
        double u_a = -k_gain * x;
        double u = alpha * u_h + (1.0 - alpha) * u_a;

        // Saturation
        if (u > U_max) u = U_max;
        if (u < -U_max) u = -U_max;

        x = x + T * u;
        t = t + T;
    }

    std::cout << "Final position: " << x << std::endl;
    return 0;
}
      
