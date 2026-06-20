
#include <iostream>
#include <vector>
#include <Eigen/Dense>

int main() {
    using Eigen::Matrix2d;
    using Eigen::Vector2d;
    using Eigen::RowVector2d;

    // Model parameters
    double M = 2.0;
    double b = 0.8;
    double k = 5.0;

    Matrix2d A;
    A << 0.0, 1.0,
          -k / M, -b / M;

    Vector2d B;
    B << 0.0,
          1.0 / M;

    RowVector2d C;
    C << 1.0, 0.0;

    RowVector2d K;
    K << 27.0, 13.6;

    Vector2d L;
    L << 28.4,
          242.14;

    double dt = 1e-3;
    double T_end = 5.0;
    int N = static_cast<int>(T_end / dt);

    double q_ref = 0.5;
    Vector2d x_ref;
    x_ref << q_ref, 0.0;

    Vector2d x;      // true state
    Vector2d x_hat;  // estimated state
    x.setZero();
    x_hat.setZero();

    std::vector<double> t_log;
    std::vector<double> q_log;
    std::vector<double> q_hat_log;
    std::vector<double> u_log;
    t_log.reserve(N);
    q_log.reserve(N);
    q_hat_log.reserve(N);
    u_log.reserve(N);

    for (int k_step = 0; k_step != N; ++k_step) {
        double t = k_step * dt;

        double y = C * x;
        double y_hat = C * x_hat;
        double y_err = y - y_hat;

        Vector2d x_hat_minus_ref = x_hat - x_ref;
        double u = -(K * x_hat_minus_ref)(0);

        t_log.push_back(t);
        q_log.push_back(y);
        q_hat_log.push_back(y_hat);
        u_log.push_back(u);

        Vector2d x_dot = A * x + B * u;
        Vector2d x_hat_dot = A * x_hat + B * u + L * y_err;

        x += dt * x_dot;
        x_hat += dt * x_hat_dot;
    }

    // Example: print final state
    std::cout << "Final q = " << q_log.back()
              << ", final q_hat = " << q_hat_log.back() << std::endl;

    return 0;
}
