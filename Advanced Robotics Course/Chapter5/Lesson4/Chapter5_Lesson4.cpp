#include <cmath>
#include <vector>
#include <iostream>

struct State {
    double x;
    double y;
    double theta;
};

struct Control {
    double v;
    double w;
};

State dynamics(const State& s, const Control& u) {
    State dx;
    dx.x = u.v * std::cos(s.theta);
    dx.y = u.v * std::sin(s.theta);
    dx.theta = u.w;
    return dx;
}

State eulerStep(const State& s, const Control& u, double dt) {
    State dx = dynamics(s, u);
    State sn;
    sn.x = s.x + dt * dx.x;
    sn.y = s.y + dt * dx.y;
    sn.theta = s.theta + dt * dx.theta;
    return sn;
}

bool withinBounds(const Control& u,
                  double v_max,
                  double w_max) {
    if (std::fabs(u.v) > v_max) return false;
    if (std::fabs(u.w) > w_max) return false;
    return true;
}

bool isStateValid(const State& s) {
    // Example: bounded workspace
    if (std::fabs(s.x) > 5.0) return false;
    if (std::fabs(s.y) > 5.0) return false;
    return true;
}

bool simulateTrajectory(const State& x0,
                        const std::vector<Control>& controls,
                        double dt,
                        double v_max,
                        double w_max,
                        std::vector<State>& out_traj) {
    out_traj.clear();
    out_traj.push_back(x0);
    State x = x0;
    for (const auto& u : controls) {
        if (!withinBounds(u, v_max, w_max)) {
            return false;
        }
        State xn = eulerStep(x, u, dt);
        if (!isStateValid(xn)) {
            return false;
        }
        out_traj.push_back(xn);
        x = xn;
    }
    return true;
}

int main() {
    State x0{0.0, 0.0, 0.0};
    double dt = 0.1;
    double v_max = 1.0;
    double w_max = 2.0;

    std::vector<Control> controls(50, Control{0.8, 1.0});
    std::vector<State> traj;

    bool feasible = simulateTrajectory(x0, controls, dt, v_max, w_max, traj);
    std::cout << "Feasible: " << feasible << std::endl;
    if (feasible) {
        const State& xf = traj.back();
        std::cout << "Final state: "
                  << xf.x << " "
                  << xf.y << " "
                  << xf.theta << std::endl;
    }
    return 0;
}
      
