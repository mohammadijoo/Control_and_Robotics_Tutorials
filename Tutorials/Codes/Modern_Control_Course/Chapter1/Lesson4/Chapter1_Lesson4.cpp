#include <Eigen/Dense>
#include <iostream>

int main() {
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    // System matrices
    MatrixXd A(2, 2);
    MatrixXd B(2, 2);
    A << 0.0,  1.0,
        -4.0, -2.0;
    B << 1.0, 0.0,
         0.0, 1.0;

    VectorXd x(2);
    VectorXd u(2);
    x << 0.0, 0.0;   // initial state
    u << 1.0, 0.0;   // step in u1, u2 = 0

    double dt = 0.001;
    int steps = 10000;

    for (int k = 0; k < steps; ++k) {
        VectorXd xdot = A * x + B * u;
        x = x + dt * xdot;  // forward Euler step

        if (k % 1000 == 0) {
            std::cout << "t=" << k * dt
                      << "  x1=" << x(0)
                      << "  x2=" << x(1) << std::endl;
        }
    }

    return 0;
}
      
