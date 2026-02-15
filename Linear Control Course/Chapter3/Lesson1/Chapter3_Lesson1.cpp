#include <iostream>
#include <Eigen/Dense>

int main() {
    // Physical parameters
    const double m = 1.0;
    const double b = 0.4;
    const double k = 2.0;

    // Continuous-time matrices
    Eigen::Matrix2d A;
    A << 0.0,      1.0,
          -k/m, -b/m;

    Eigen::Vector2d B;
    B << 0.0,
          1.0/m;

    Eigen::Vector2d x;  // state [q; q_dot]
    x << 0.0, 0.0;

    const double u = 1.0;   // constant input (step force)
    const double dt = 0.001;
    const int steps = 10000;

    for (int i = 0; i < steps; ++i) {
        Eigen::Vector2d xdot = A * x + B * u;
        x += dt * xdot;  // Euler integration
    }

    std::cout << "Final position approx: " << x(0) << std::endl;

    // In a ROS(2) robotic joint controller, one would embed this model in a real-time loop
    // and update x using measured torques and velocities, using ros_control or ros2_control.
    return 0;
}
