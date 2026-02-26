#include <iostream>
#include <cmath>

struct State {
    double theta;
    double omega;
};

double Kp = 5.0;
double Kd = 1.0;
double dt = 0.02;

double policy(const State& x) {
    return -Kp * x.theta - Kd * x.omega;
}

State step(const State& x, double u, double c) {
    State xn;
    xn.theta = x.theta + dt * x.omega;
    xn.omega = x.omega + dt * (u - c * x.omega);
    return xn;
}

double rollout(double c, int horizon = 500) {
    State x{0.5, 0.0};
    double gamma = 0.99;
    double total_reward = 0.0;
    double pow_gamma = 1.0;
    for (int t = 0; t < horizon; ++t) {
        double u = policy(x);
        double cost = x.theta * x.theta
                      + 0.1 * x.omega * x.omega
                      + 0.01 * u * u;
        total_reward += pow_gamma * (-cost);
        pow_gamma *= gamma;
        x = step(x, u, c);
    }
    return total_reward;
}

int main() {
    double J_sim  = rollout(0.1);
    double J_real = rollout(0.3);
    std::cout << "J_sim  = " << J_sim  << std::endl;
    std::cout << "J_real = " << J_real << std::endl;
    std::cout << "Gap   = " << (J_real - J_sim) << std::endl;
    return 0;
}
      
