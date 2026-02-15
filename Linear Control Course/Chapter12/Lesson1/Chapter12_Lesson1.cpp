#include <iostream>
#include <vector>

int main() {
    // PID gains from the design example
    double Kp = 43.47;
    double Ki = 91.76;
    double Kd = 11.0;

    // Simulation parameters
    double dt = 0.001;
    double T_end = 10.0;
    int N = static_cast<int>(T_end / dt);

    // Reference (unit step)
    auto r = [](double /*t*/) { return 1.0; };

    // State of plant G(s) = 1/(s*(s+1))
    // State-space realization:
    // x1_dot = x2
    // x2_dot = -x2 - 0*x1 + u
    double x1 = 0.0;  // output position y
    double x2 = 0.0;  // velocity
    double y  = 0.0;

    // PID states
    double e_prev = 0.0;
    double I      = 0.0;

    std::vector<double> t_vec, y_vec;

    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        double ref = r(t);
        y = x1;

        double e = ref - y;
        I += e * dt;
        double D = (e - e_prev) / dt;
        double u = Kp * e + Ki * I + Kd * D;
        e_prev = e;

        // Simple Euler integration of plant
        double x1_dot = x2;
        double x2_dot = -x2 + u; // corresponds roughly to 1/(s*(s+1))
        x1 += x1_dot * dt;
        x2 += x2_dot * dt;

        t_vec.push_back(t);
        y_vec.push_back(y);
    }

    // Print a few samples
    for (size_t i = 0; i < t_vec.size(); i += 1000) {
        std::cout << "t=" << t_vec[i]
                  << " y=" << y_vec[i] << std::endl;
    }

    return 0;
}
