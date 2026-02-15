#include <iostream>
#include <vector>
#include <cmath>

struct Metrics {
    double ess;
    double ts;
    double umax;
};

Metrics simulateFirstOrder(double K, double tFinal = 10.0, double dt = 1e-3) {
    int nSteps = static_cast<int>(tFinal / dt);
    std::vector<double> y(nSteps + 1), u(nSteps + 1), t(nSteps + 1);

    double x = 0.0;
    double r = 1.0;
    for (int k = 0; k <= nSteps; ++k) {
        t[k] = k * dt;
        double e = r - x;
        u[k] = K * e;
        y[k] = x;
        double xdot = -x + u[k];
        x += dt * xdot;
    }

    double yss = y.back();
    double ess = r - yss;
    double tol = 0.02 * std::fabs(yss);
    double ts = tFinal;
    for (int k = 0; k <= nSteps; ++k) {
        bool ok = true;
        for (int j = k; j <= nSteps; ++j) {
            if (std::fabs(y[j] - yss) > tol) {
                ok = false;
                break;
            }
        }
        if (ok) {
            ts = t[k];
            break;
        }
    }

    double umax = 0.0;
    for (int k = 0; k <= nSteps; ++k) {
        umax = std::max(umax, std::fabs(u[k]));
    }
    return {ess, ts, umax};
}

int main() {
    double gains[] = {1.0, 2.0, 5.0, 10.0};
    for (double K : gains) {
        Metrics m = simulateFirstOrder(K);
        std::cout << "K = " << K
                  << " | ess = " << m.ess
                  << ", ts = " << m.ts
                  << ", umax = " << m.umax
                  << std::endl;
    }
    return 0;
}
