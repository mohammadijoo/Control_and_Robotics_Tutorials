#include <iostream>
#include <vector>

int main() {
    // Physical parameters
    double m  = 1000.0;   // kg
    double b  = 50.0;     // N*s/m
    double Ku = 500.0;    // N per control unit

    // PI gains (for example, from Python design)
    double Kp = 0.8;
    double Ki = 0.4;

    // Simulation parameters
    double h   = 0.01;   // time step [s]
    double T   = 20.0;   // total simulation time [s]
    int    N   = static_cast<int>(T / h);

    // Reference speed (step)
    double v_ref = 10.0; // m/s

    // State variables
    double v  = 0.0;     // current speed
    double ei = 0.0;     // integral of error

    std::vector<double> time;
    std::vector<double> v_hist;

    for (int k = 0; k <= N; ++k) {
        double t = k * h;
        double e = v_ref - v;
        ei += e * h;  // integrate error

        // PI controller
        double u_ctrl = Kp * e + Ki * ei;  // control signal (normalized)

        // Plant dynamics: m dv/dt = Ku*u_ctrl - b*v
        double dv = (Ku * u_ctrl - b * v) / m;

        // Euler integration
        v += dv * h;

        time.push_back(t);
        v_hist.push_back(v);
    }

    // Print some samples
    for (size_t i = 0; i < time.size(); i += 200) {
        std::cout << "t = " << time[i]
                  << " s, v = " << v_hist[i] << " m/s" << std::endl;
    }

    return 0;
}
