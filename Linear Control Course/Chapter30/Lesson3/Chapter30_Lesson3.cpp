#include <iostream>
#include <vector>
#include <algorithm>

double sat(double u, double u_max) {
    return std::max(-u_max, std::min(u, u_max));
}

int main() {
    double k = 5.0;
    double D = 3.0;
    double u_max = 1.0;
    double x0 = 0.0;
    double t_final = 20.0;
    double dt = 0.001;

    int N = static_cast<int>(t_final / dt);
    std::vector<double> t(N + 1), x(N + 1), u_cmd(N + 1), u_applied(N + 1);

    x[0] = x0;
    t[0] = 0.0;

    for (int n = 0; n < N; ++n) {
        t[n + 1] = t[n] + dt;

        u_cmd[n] = -k * x[n];
        u_applied[n] = sat(u_cmd[n], u_max);

        double x_dot = -x[n] + u_applied[n] + D;
        x[n + 1] = x[n] + dt * x_dot;
    }

    u_cmd[N] = -k * x[N];
    u_applied[N] = sat(u_cmd[N], u_max);

    // Print a few samples
    for (int n = 0; n <= N; n += N / 10) {
        std::cout << "t=" << t[n]
                  << "  x=" << x[n]
                  << "  u=" << u_applied[n] << std::endl;
    }

    return 0;
}
