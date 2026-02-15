#include <iostream>
#include <Eigen/Dense>
// In a robotic context, this could be part of a ROS control loop:
// #include <ros/ros.h>

struct LagCompensator {
    double Kc;
    double beta;
    double T;
    double h;      // sampling period
    double a1;
    double b0;
    double b1;
    double u_prev;
    double e_prev;

    LagCompensator(double Kc_, double beta_, double T_, double h_)
        : Kc(Kc_), beta(beta_), T(T_), h(h_),
          u_prev(0.0), e_prev(0.0)
    {
        // Backward Euler discretization of:
        // beta T du/dt + u = Kc T de/dt + Kc e
        // After algebra, one obtains:
        double denom = beta * T / h + 1.0;
        a1 = (beta * T / h) / denom;
        b0 = (Kc * (T / h + 1.0)) / denom;
        b1 = (-Kc * (T / h)) / denom;
    }

    double update(double e) {
        double u = a1 * u_prev + b0 * e + b1 * e_prev;
        u_prev = u;
        e_prev = e;
        return u;
    }
};

int main() {
    LagCompensator lag(5.0, 5.0, 1.0, 0.001); // Kc, beta, T, h
    double e = 0.1; // example tracking error
    for (int k = 0; k < 10; ++k) {
        double u = lag.update(e);
        std::cout << "k=" << k << " u=" << u << std::endl;
    }
    return 0;
}
