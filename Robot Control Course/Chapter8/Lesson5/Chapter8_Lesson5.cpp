
#include <iostream>
#include <cmath>

struct Params {
    double theta1{2.0};
    double theta2{0.4};
    double theta3{5.0};
};

struct AdaptiveState {
    double q{0.0};
    double dq{0.0};
    double th1_hat{1.5};
    double th2_hat{0.2};
    double th3_hat{4.0};
};

double lam = 5.0;
double k_s = 10.0;
double gamma1 = 5.0, gamma2 = 5.0, gamma3 = 5.0;
double A = 0.5, omega = 1.0;

void desired_traj(double t, double &qd, double &dqd, double &ddqd) {
    qd = A * std::sin(omega * t);
    dqd = A * omega * std::cos(omega * t);
    ddqd = -A * omega * omega * std::sin(omega * t);
}

void regressor(double q, double dq, double qd, double dqd, double ddqd,
               double &Y1, double &Y2, double &Y3, double &s) {
    double e = q - qd;
    double de = dq - dqd;
    double dq_r = dqd - lam * e;
    double ddq_r = ddqd - lam * de;
    Y1 = ddq_r;
    Y2 = dq_r;
    Y3 = std::sin(q);
    s = dq - dq_r;
}

double plant_accel(const Params &p, double q, double dq, double tau) {
    return (tau - p.theta2 * dq - p.theta3 * std::sin(q)) / p.theta1;
}

int main() {
    Params plant;
    AdaptiveState x;
    double dt = 0.001;
    double T = 20.0;

    for (double t = 0.0; t <= T; t += dt) {
        double qd, dqd, ddqd;
        desired_traj(t, qd, dqd, ddqd);

        double Y1, Y2, Y3, s;
        regressor(x.q, x.dq, qd, dqd, ddqd, Y1, Y2, Y3, s);

        // Adaptive control
        double tau = Y1 * x.th1_hat + Y2 * x.th2_hat + Y3 * x.th3_hat - k_s * s;

        // Plant dynamics
        double ddq = plant_accel(plant, x.q, x.dq, tau);

        // Euler integration
        x.q  += dt * x.dq;
        x.dq += dt * ddq;

        // Parameter update
        x.th1_hat += dt * (-gamma1 * Y1 * s);
        x.th2_hat += dt * (-gamma2 * Y2 * s);
        x.th3_hat += dt * (-gamma3 * Y3 * s);

        // Logging (e.g., print every few steps or store in arrays)
        if (static_cast<int>(t / dt) % 1000 == 0) {
            std::cout << t << " " << x.q << std::endl;
        }
    }
    return 0;
}
