#include <iostream>
#include <vector>
#include <Eigen/Dense>

using Eigen::VectorXd;

// Discrete-time input generators
VectorXd step_input(const VectorXd& t) {
    VectorXd u(t.size());
    for (int i = 0; i < t.size(); ++i) {
        u(i) = (t(i) >= 0.0) ? 1.0 : 0.0;
    }
    return u;
}

VectorXd ramp_input(const VectorXd& t) {
    VectorXd u(t.size());
    for (int i = 0; i < t.size(); ++i) {
        u(i) = t(i);
    }
    return u;
}

// LTI gain system: y = k * u
VectorXd S_gain(const VectorXd& u, double k = 3.0) {
    return k * u;
}

// LTV gain system: y = t * u
VectorXd S_time_varying(const VectorXd& u, const VectorXd& t) {
    VectorXd y(t.size());
    for (int i = 0; i < t.size(); ++i) {
        y(i) = t(i) * u(i);
    }
    return y;
}

double linearity_error_gain(const VectorXd& t) {
    double alpha = 1.2, beta = -0.8;
    VectorXd u1 = step_input(t);
    VectorXd u2 = ramp_input(t);
    VectorXd u_combo = alpha * u1 + beta * u2;

    VectorXd y_combo = S_gain(u_combo);
    VectorXd y_lin = alpha * S_gain(u1) + beta * S_gain(u2);

    return (y_combo - y_lin).norm();
}

int main() {
    int N = 501;
    VectorXd t(N);
    double t0 = 0.0, tf = 5.0;
    double dt = (tf - t0) / (N - 1);
    for (int i = 0; i < N; ++i) {
        t(i) = t0 + i * dt;
    }

    double err_gain = linearity_error_gain(t);
    std::cout << "Linearity error (LTI gain): " << err_gain << std::endl;

    // For robotics: this pattern would be embedded in a controller update loop
    // with u as actuator commands and y as torque or velocity.
    return 0;
}
