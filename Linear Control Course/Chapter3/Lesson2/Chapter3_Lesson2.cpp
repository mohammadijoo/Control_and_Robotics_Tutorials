#include <iostream>
#include <Eigen/Dense>

int main() {
    // Physical parameters
    double m1 = 1.0;
    double m2 = 0.5;
    double b1 = 0.4;
    double b2 = 0.2;
    double k1 = 4.0;
    double k2 = 2.0;

    // Mass, damping, stiffness matrices (2x2)
    Eigen::Matrix2d M;
    Eigen::Matrix2d B;
    Eigen::Matrix2d K;

    M << m1, 0.0,
          0.0, m2;

    B << b1 + b2, -b2,
          -b2,      b2;

    K << k1 + k2, -k2,
          -k2,      k2;

    // State: y = [y1, y2], v = [v1, v2]
    Eigen::Vector2d y;
    Eigen::Vector2d v;

    y << 0.0, 0.0;  // initial displacements
    v << 0.0, 0.0;  // initial velocities

    double dt = 0.001;        // integration step [s]
    double t_final = 5.0;
    int steps = static_cast<int>(t_final / dt);

    for (int k = 0; k < steps; ++k) {
        double t = k * dt;

        // External force applied to mass 1 (unit step)
        Eigen::Vector2d f;
        f << 1.0, 0.0;

        // Acceleration: M * a = f - B * v - K * y
        Eigen::Vector2d a = M.ldlt().solve(f - B * v - K * y);

        // Explicit Euler integration
        v = v + dt * a;
        y = y + dt * v;

        // Print a few samples
        if (k % 1000 == 0) {
            std::cout << "t = " << t
                      << "  y1 = " << y(0)
                      << "  y2 = " << y(1) << std::endl;
        }
    }

    return 0;
}
