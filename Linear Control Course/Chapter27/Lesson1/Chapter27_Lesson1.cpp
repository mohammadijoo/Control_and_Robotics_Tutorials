#include <iostream>
#include <vector>

int main() {
    double tau = 0.5;
    double Kp  = 4.0;
    double Ki  = 5.0;

    double h   = 0.001;   // sampling period [s]
    double T   = 5.0;     // total simulation time
    int N      = static_cast<int>(T / h);

    std::vector<double> t(N), y_track(N), y_reg(N);

    // Tracking scenario: step in r, no disturbance
    double x = 0.0;
    double u = 0.0;
    double e_prev = 0.0;
    for (int k = 0; k < N; ++k) {
        double tk = k * h;
        t[k] = tk;

        double r = 1.0;    // step reference
        double d = 0.0;    // no disturbance

        double y = x + d;
        double e = r - y;

        // PI control (incremental form)
        double du = Kp * (e - e_prev) + Ki * h * e;
        u += du;
        e_prev = e;

        // Plant integration (forward Euler)
        double dx = (-1.0 / tau) * x + (1.0 / tau) * u;
        x += h * dx;

        y_track[k] = y;
    }

    // Regulation scenario: step disturbance at plant output, r = 0
    x = 0.0;
    u = 0.0;
    e_prev = 0.0;
    for (int k = 0; k < N; ++k) {
        double tk = k * h;
        double r = 0.0;
        double d = (tk >= 1.0) ? 0.5 : 0.0;  // step disturbance at t = 1 s

        double y = x + d;
        double e = r - y;

        double du = Kp * (e - e_prev) + Ki * h * e;
        u += du;
        e_prev = e;

        double dx = (-1.0 / tau) * x + (1.0 / tau) * u;
        x += h * dx;

        y_reg[k] = y;
    }

    // Print a few samples (in practice, log to file or plot offline)
    for (int k = 0; k < N; k += N / 10) {
        std::cout << "t = " << t[k]
                  << ", y_track = " << y_track[k]
                  << ", y_reg = " << y_reg[k]
                  << std::endl;
    }

    return 0;
}
