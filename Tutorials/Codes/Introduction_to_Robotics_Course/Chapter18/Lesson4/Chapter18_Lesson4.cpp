#include <iostream>

int main() {
    double dt = 0.001;
    double T  = 5.0;
    int steps = static_cast<int>(T / dt);

    double a_true = 2.0;
    double K      = 10.0;
    double gamma  = 5.0;

    auto x_ref = [](double t) {
        return (t > 0.5) ? 1.0 : 0.0;
    };

    double x     = 0.0;
    double a_hat = 0.0;

    for (int k = 0; k < steps; ++k) {
        double t  = k * dt;
        double xr = x_ref(t);
        double e  = x - xr;

        double u_fb = -K * e;
        double u_ff = a_hat * x;
        double u    = u_fb + u_ff;

        // Plant dynamics
        double x_dot = u - a_true * x;
        x += dt * x_dot;

        // Adaptive update
        a_hat -= gamma * e * x * dt;
    }

    std::cout << "Final a_hat = " << a_hat << std::endl;
    return 0;
}
      
