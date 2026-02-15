#include <iostream>

struct PIController {
    double kp;
    double ki;
    double xi;
    double Ts;
    PIController(double kp_, double ki_, double Ts_)
        : kp(kp_), ki(ki_), xi(0.0), Ts(Ts_) {}
    double update(double ref, double meas) {
        double e = ref - meas;
        xi += e * Ts;
        return kp * e + ki * xi;
    }
};

int main() {
    double Ts = 1e-4;
    int N = 2000;

    // PI loops
    PIController current_loop(50.0, 5000.0, Ts);
    PIController speed_loop(2.0, 200.0, Ts);
    PIController position_loop(20.0, 200.0, Ts);

    // Motor states
    double i = 0.0;
    double w = 0.0;
    double theta = 0.0;

    // Parameters
    double J = 0.01;
    double B = 0.001;
    double R = 0.5;
    double L = 1e-3;
    double Kt = 0.05;
    double Ke = 0.05;

    double theta_ref = 1.57; // rad

    for (int k = 0; k < N; ++k) {
        double w_ref = position_loop.update(theta_ref, theta);
        double i_ref = speed_loop.update(w_ref, w);
        double v = current_loop.update(i_ref, i);

        // Simple Euler integration of motor model
        double di = (-R * i + v - Ke * w) / L;
        double dw = (-B * w + Kt * i) / J;
        double dtheta = w;

        i += di * Ts;
        w += dw * Ts;
        theta += dtheta * Ts;
    }

    std::cout << "Final theta [rad]: " << theta << std::endl;
    return 0;
}
