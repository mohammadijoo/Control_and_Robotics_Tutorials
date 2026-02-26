#include <iostream>
#include <vector>

int main() {
    const double T = 0.5;
    const double zeta = 0.7;
    const double ts_des = 1.0;
    const double wn = 4.0 / (zeta * ts_des);

    const double Kp = 0.5 * wn * wn;
    const double Td = (zeta * wn - 1.0) / Kp;

    std::cout << "Kp = " << Kp << ", Td = " << Td << std::endl;

    double h = 1e-3;     // integration time step
    double t_end = 5.0;
    int N = static_cast<int>(t_end / h);

    // state: x1 = theta, x2 = theta_dot
    double x1 = 0.0;
    double x2 = 0.0;
    double r = 1.0;      // unit step reference

    std::vector<double> t_vec;
    std::vector<double> y_vec;

    t_vec.reserve(N + 1);
    y_vec.reserve(N + 1);

    t_vec.push_back(0.0);
    y_vec.push_back(x1);

    for (int k = 0; k < N; ++k) {
        double t = (k + 1) * h;

        // PD control law: u = Kp * (e - Td * theta_dot)
        double e = r - x1;
        double u = Kp * (e - Td * x2);

        // continuous-time dynamics
        double x1_dot = x2;
        double x2_dot = (-x2 + u) / T;

        // Euler integration
        x1 += h * x1_dot;
        x2 += h * x2_dot;

        t_vec.push_back(t);
        y_vec.push_back(x1);
    }

    // Print a few samples or export to file
    for (size_t k = 0; k < t_vec.size(); k += 500) {
        std::cout << t_vec[k] << "  " << y_vec[k] << std::endl;
    }

    return 0;
}
