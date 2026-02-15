#include <iostream>
#include <vector>
#include <cmath>

int main() {
    // Simple DC motor parameters
    double J  = 0.01;
    double b  = 0.1;
    double K  = 1.0;

    // PI gains from loop-shaping considerations
    double Kp = 20.0;
    double Ki = 40.0;

    double dt    = 0.0005;  // integration step
    double t_end = 1.0;     // simulation horizon

    // States: x1 = position, x2 = velocity
    double x1 = 0.0;
    double x2 = 0.0;
    double integral_error = 0.0;
    double ref = 1.0;       // unit-step reference

    int steps = static_cast<int>(t_end / dt);
    std::vector<double> time(steps), y(steps);

    for (int k = 0; k < steps; ++k) {
        double t = k * dt;
        double e = ref - x1;              // position error
        integral_error += e * dt;
        double u = Kp * e + Ki * integral_error; // PI control torque command

        // Motor dynamics: J * x2_dot + b * x2 = K * u
        double x2_dot = (K * u - b * x2) / J;
        double x1_dot = x2;

        x1 += dt * x1_dot;
        x2 += dt * x2_dot;

        time[k] = t;
        y[k]    = x1;
    }

    // Print final position as a simple check
    std::cout << "Final position: " << y.back() << std::endl;

    // In a ROS-based robot, similar PI computations appear inside a
    // position controller node using libraries such as 'ros_control',
    // while plant dynamics are provided by the robot's URDF and dynamics engine.
    return 0;
}
