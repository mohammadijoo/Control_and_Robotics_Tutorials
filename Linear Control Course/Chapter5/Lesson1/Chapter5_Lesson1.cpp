#include <iostream>
#include <vector>

int main() {
    double K = 2.0;
    double tau = 0.5;
    double dt = 0.001;           // time step [s]
    double T_final = 2.0;
    int N = static_cast<int>(T_final / dt);

    std::vector<double> t(N + 1);
    std::vector<double> y(N + 1);

    // Initial conditions
    t[0] = 0.0;
    y[0] = 0.0;

    // Unit-step input u(t) = 1
    for (int k = 0; k < N; ++k) {
        t[k + 1] = (k + 1) * dt;
        double u = 1.0;
        double dy_dt = (K * u - y[k]) / tau;   // from tau dy/dt + y = K u
        y[k + 1] = y[k] + dt * dy_dt;
    }

    // Print a few samples
    for (int k = 0; k <= N; k += N / 10) {
        std::cout << "t = " << t[k]
                  << " s, y = " << y[k] << std::endl;
    }

    return 0;
}
