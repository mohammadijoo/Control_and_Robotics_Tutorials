#include <iostream>
#include <vector>
#include <cmath>

struct State {
    double x; // position
    double v; // velocity
};

std::vector<State> propagateDoubleIntegrator(
        const State& x0, double u, double dt, double T_step, double u_max)
{
    double u_sat = std::max(-u_max, std::min(u, u_max));
    State x = x0;
    std::vector<State> traj;
    traj.push_back(x);

    int N = static_cast<int>(T_step / dt);
    for (int k = 0; k < N; ++k) {
        // Euler integration
        State xdot;
        xdot.x = x.v;
        xdot.v = u_sat;

        x.x += dt * xdot.x;
        x.v += dt * xdot.v;

        traj.push_back(x);
    }
    return traj;
}

int main() {
    State x0{0.0, 0.0};
    double u = 0.8;
    double dt = 0.01;
    double T_step = 0.5;
    double u_max = 1.0;

    auto traj = propagateDoubleIntegrator(x0, u, dt, T_step, u_max);
    std::cout << "Final state: x = "
              << traj.back().x
              << ", v = " << traj.back().v << std::endl;
    return 0;
}
      
