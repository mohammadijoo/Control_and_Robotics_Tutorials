#include <iostream>
#include <vector>

struct PDServo {
    double d;   // viscous damping
    double k;   // stiffness
    double Kp;  // proportional gain
    double Kd;  // derivative gain

    // simulate closed-loop response for given mass m
    std::vector<double> simulate(double m, double t_end, double dt) const {
        double x = 0.0;   // position
        double v = 0.0;   // velocity
        double r = 1.0;   // unit step reference
        std::vector<double> y;
        y.reserve(static_cast<std::size_t>(t_end / dt) + 1);

        for (double t = 0.0; t <= t_end; t += dt) {
            double e = r - x;
            double edot = -v; // derivative of error (dr/dt = 0 for step)
            double u = Kp * e + Kd * edot;

            // mass-spring-damper dynamics: m x'' + d x' + k x = u
            double a = (u - d * v - k * x) / m;
            v += a * dt;
            x += v * dt;
            y.push_back(x);
        }
        return y;
    }
};

int main() {
    PDServo servo{2.0, 50.0, 150.0, 20.0};

    std::vector<double> masses{0.5, 1.0, 2.0};
    for (double m : masses) {
        auto y = servo.simulate(m, 5.0, 0.001);
        std::cout << "Final value for m = " << m
                  << " is " << y.back() << std::endl;
    }
    return 0;
}
