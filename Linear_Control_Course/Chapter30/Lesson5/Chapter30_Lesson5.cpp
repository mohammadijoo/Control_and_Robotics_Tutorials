#include <iostream>
#include <array>
#include <Eigen/Dense>

int main() {
    // 2-state mass-spring-damper
    Eigen::Matrix2d A;
    A << 0.0, 1.0,
          -2.0, -0.5;
    Eigen::Vector2d B;
    B << 0.0,
          1.0;

    // Precomputed LQR gain K (row vector)
    Eigen::RowVector2d K;
    K << 3.0, 1.2;

    // Current state x
    Eigen::Vector2d x;
    x << 0.1,
          0.0;

    // State-feedback control law u = -K x
    double u = - (K * x)(0);

    std::cout << "Control input u = " << u << std::endl;

    // One-step Euler integration (for illustration only)
    Eigen::Vector2d xdot = A * x + B * u;
    double dt = 0.001;
    Eigen::Vector2d x_next = x + dt * xdot;

    std::cout << "Next state x_next = ["
              << x_next(0) << ", " << x_next(1) << "]"
              << std::endl;
    return 0;
}
