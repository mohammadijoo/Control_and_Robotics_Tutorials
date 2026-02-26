
#include <iostream>
#include <algorithm>  // std::max, std::min

// Safety filter for dot{x} = u, with h(x) = x and input box [u_min, u_max].
double safe_input(double x, double u_des, double u_min, double u_max, double kappa) {
    double u_low = std::max(u_min, -kappa * x);
    double u_high = u_max;
    double u_safe = std::min(u_high, std::max(u_low, u_des));
    return u_safe;
}

int main() {
    double x = 0.1;      // initial state, assume x >= 0
    double u_min = -2.0;
    double u_max =  2.0;
    double kappa = 5.0;
    double k_p = 5.0;
    double dt = 0.001;
    double T = 2.0;

    int N = static_cast<int>(T / dt);

    for (int k = 0; k < N; ++k) {
        double t = (k + 1) * dt;
        double x_ref = 1.0;
        double u_des = -k_p * (x - x_ref);
        double u_safe = safe_input(x, u_des, u_min, u_max, kappa);
        x = x + dt * u_safe;

        if (k % 100 == 0) {
            std::cout << "t = " << t
                      << ", x = " << x
                      << ", u_des = " << u_des
                      << ", u_safe = " << u_safe << std::endl;
        }
    }

    return 0;
}
