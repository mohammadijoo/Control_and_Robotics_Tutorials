#include <iostream>
#include <vector>

// Simple state-space realization for G(s) = K / (s (s + 1))
// x_dot = A x + B u,  y = C x
// One possible realization:
//   x = [x1; x2]
//   x1_dot = x2
//   x2_dot = -x2 + K * u
//   y = x1
int main() {
    double K = 5.0;
    double dt = 0.001;
    double t_end = 10.0;
    int N = static_cast<int>(t_end / dt);

    double x1 = 0.0;  // position-like state
    double x2 = 0.0;  // velocity-like state

    std::vector<double> t_vec, e_step, e_ramp;

    // STEP input r(t) = 1
    x1 = 0.0;
    x2 = 0.0;
    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        double r = 1.0;        // step
        double y = x1;
        double e = r - y;
        double u = e;          // C(s) = 1 (unity controller)

        // Euler integration of the plant
        double x1_dot = x2;
        double x2_dot = -x2 + K * u;

        x1 += dt * x1_dot;
        x2 += dt * x2_dot;

        t_vec.push_back(t);
        e_step.push_back(e);
    }

    // RAMP input r(t) = t
    x1 = 0.0;
    x2 = 0.0;
    e_ramp.resize(N);
    for (int k = 0; k < N; ++k) {
        double t = k * dt;
        double r = t;          // ramp
        double y = x1;
        double e = r - y;
        double u = e;

        double x1_dot = x2;
        double x2_dot = -x2 + K * u;

        x1 += dt * x1_dot;
        x2 += dt * x2_dot;

        e_ramp[k] = e;
    }

    std::cout << "# t  e_step  e_ramp\n";
    for (int k = 0; k < N; ++k) {
        std::cout << t_vec[k] << " "
                  << e_step[k] << " "
                  << e_ramp[k] << "\n";
    }

    // In a real robot controller using ROS, a similar loop runs at a fixed
    // control period dt on a joint_state callback, computing the error e and
    // sending the control input u via ros_control interfaces.
    return 0;
}
