#include <iostream>
#include <vector>

int main() {
    // Plant: x_dot = -(1/tau) * x + (K/tau) * u
    double K = 1.0;
    double tau = 0.5;

    // PI gains (for example, from analytic design)
    double Kp = 1.1;
    double Ki = 3.0;

    double dt = 0.001;      // sampling period [s]
    int    N  = 5000;       // number of steps (5 s)

    double x = 0.0;         // plant state (output y = x)
    double integral_e = 0.0;
    double r = 1.0;         // reference (step)
    double u = 0.0;         // control input

    std::vector<double> t_vec, y_vec, e_vec;

    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        double y = x;
        double e = r - y;

        // PI controller
        integral_e += e * dt;
        u = Kp * e + Ki * integral_e;

        // Plant integration (forward Euler)
        double x_dot = -(1.0 / tau) * x + (K / tau) * u;
        x += x_dot * dt;

        t_vec.push_back(t);
        y_vec.push_back(y);
        e_vec.push_back(e);
    }

    std::cout << "Final output y(T) = " << y_vec.back() << std::endl;
    std::cout << "Final error e(T)  = " << e_vec.back() << std::endl;
    return 0;
}
