
#include <iostream>
#include <random>
#include <Eigen/Dense>

int main() {
    using namespace Eigen;

    Matrix2d A;
    A << 0.0, 1.0,
          0.0, -0.5;
    Vector2d B;
    B << 0.0, 1.0;
    RowVector2d C;
    C << 1.0, 0.0;
    RowVector2d K;
    K << 3.0, 2.0;
    Vector2d L;
    L << 5.0, 6.0;

    double Ts = 0.001;
    int steps = 5000;

    Vector2d x;      x << 0.5, 0.0;
    Vector2d x_hat;  x_hat.setZero();

    std::mt19937 gen(0);
    std::normal_distribution<double> dist(0.0, 0.01);

    for (int k = 0; k < steps; ++k) {
        double t = k * Ts;

        double y = C * x + dist(gen);
        double y_hat = C * x_hat;
        double u = -K * x_hat;

        Vector2d x_dot = A * x + B * u;
        Vector2d x_hat_dot = A * x_hat + B * u + L * (y - y_hat);

        x += Ts * x_dot;
        x_hat += Ts * x_hat_dot;

        if (k % 1000 == 0) {
            std::cout << "t=" << t
                      << "  q=" << x(0)
                      << "  q_hat=" << x_hat(0)
                      << std::endl;
        }
    }
    return 0;
}
