#include <iostream>
#include <Eigen/Dense>

int main() {
    using namespace Eigen;

    // Matrices (2 states, 1 input)
    Matrix2d A;
    A << 0.0, 1.0,
          -2.0, -0.4;   // assume m = 1, k = 2, c = 0.4
    Vector2d B;
    B << 0.0,
          1.0;          // input gain = 1/m

    // Initial state: position = 0, velocity = 0
    Vector2d x;
    x << 0.0, 0.0;

    double u = 1.0;     // constant input
    double dt = 0.001;
    int steps = 10000;

    for (int k = 0; k < steps; ++k) {
        Vector2d xdot = A * x + B * u;
        x = x + dt * xdot;   // explicit Euler
    }

    std::cout << "Final state x = " << x.transpose() << std::endl;
    return 0;
}
      
