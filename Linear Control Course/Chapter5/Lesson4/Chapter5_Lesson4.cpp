#include <iostream>
#include <cmath>

// First-order metrics for step response
struct Metrics {
    double t_delay;
    double t_rise;
    double t_settle;
};

Metrics firstOrderMetrics(double tau, double tol) {
    Metrics m;
    m.t_delay = -tau * std::log(0.5);       // 50%
    m.t_rise = tau * std::log(9.0);         // 10-90%
    m.t_settle = -tau * std::log(tol);      // tol band
    return m;
}

int main() {
    double K = 2.0;
    double tau = 0.1;
    double dt = 0.001;
    double T_final = 1.0;

    Metrics m = firstOrderMetrics(tau, 0.02);
    std::cout << "Delay time t_d = " << m.t_delay << std::endl;
    std::cout << "Rise time t_r = " << m.t_rise << std::endl;
    std::cout << "Settling time t_s = " << m.t_settle << std::endl;

    // Sample analytical response y(t) = K(1 - exp(-t/tau))
    for (double t = 0.0; t <= T_final; t += dt) {
        double y = K * (1.0 - std::exp(-t / tau));
        // This loop could be integrated into robot joint control simulation
        // using libraries like ROS control_toolbox in a larger framework.
    }
    return 0;
}
