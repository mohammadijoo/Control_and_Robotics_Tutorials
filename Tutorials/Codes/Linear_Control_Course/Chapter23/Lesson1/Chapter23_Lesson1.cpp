#include <iostream>
#include <vector>
#include <random>

// Simple Euler integration of J*theta_ddot + b*theta_dot = u
struct JointState {
    double theta;      // position
    double theta_dot;  // velocity
};

void simulate_joint(double J, double b,
                    double u,  // constant torque
                    double dt, double T,
                    std::vector<JointState>& traj)
{
    JointState x{0.0, 0.0};
    int N = static_cast<int>(T / dt);
    traj.clear();
    traj.reserve(N + 1);
    traj.push_back(x);

    for (int k = 0; k < N; ++k) {
        double theta_ddot = (u - b * x.theta_dot) / J;
        x.theta_dot += dt * theta_ddot;
        x.theta     += dt * x.theta_dot;
        traj.push_back(x);
    }
}

int main() {
    double J0 = 0.01;
    double b0 = 0.05;
    double dt = 0.001;
    double T  = 4.0;
    double u  = 1.0;    // step torque

    std::mt19937 gen(1);
    std::uniform_real_distribution<double> dist(-0.2, 0.2);

    for (int i = 0; i < 5; ++i) {
        double J = J0 * (1.0 + dist(gen));
        double b = b0 * (1.0 + dist(gen));

        std::vector<JointState> traj;
        simulate_joint(J, b, u, dt, T, traj);

        std::cout << "Sample " << i
                  << ": J=" << J
                  << ", b=" << b
                  << ", final theta=" << traj.back().theta
                  << std::endl;
    }

    return 0;
}
