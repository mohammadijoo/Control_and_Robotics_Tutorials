#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using Eigen::Vector2d;

Vector2d q_path(double s) {
    // q_p(s) = [cos(pi s/2), sin(pi s/2)]^T
    double q1 = std::cos(0.5 * M_PI * s);
    double q2 = std::sin(0.5 * M_PI * s);
    return Vector2d(q1, q2);
}

Vector2d dqds_path(double s) {
    double dq1 = -0.5 * M_PI * std::sin(0.5 * M_PI * s);
    double dq2 =  0.5 * M_PI * std::cos(0.5 * M_PI * s);
    return Vector2d(dq1, dq2);
}

Vector2d d2qds2_path(double s) {
    double c = 0.5 * M_PI;
    double d2q1 = -(c * c) * std::cos(c * s);
    double d2q2 = -(c * c) * std::sin(c * s);
    return Vector2d(d2q1, d2q2);
}

double s_time(double t, double T) {
    double tau = t / T;
    return 3.0 * tau * tau - 2.0 * tau * tau * tau;
}

double sdot_time(double t, double T) {
    double tau = t / T;
    return (6.0 * tau - 6.0 * tau * tau) / T;
}

double sddot_time(double t, double T) {
    double tau = t / T;
    return (6.0 - 12.0 * tau) / (T * T);
}

int main() {
    double T = 2.0;
    int n_samples = 100;
    double qdot_max1 = 1.0;
    double qdot_max2 = 1.0;

    bool violated = false;
    for (int k = 0; k < n_samples; ++k) {
        double t = (T * k) / (n_samples - 1);
        double s = s_time(t, T);
        double sdot = sdot_time(t, T);
        double sddot = sddot_time(t, T);

        Vector2d q = q_path(s);
        Vector2d dqds = dqds_path(s);
        Vector2d d2qds2 = d2qds2_path(s);

        Vector2d qdot = dqds * sdot;
        Vector2d qddot = d2qds2 * (sdot * sdot) + dqds * sddot;

        if (std::abs(qdot(0)) > qdot_max1 || std::abs(qdot(1)) > qdot_max2) {
            violated = true;
        }
    }

    if (violated) {
        std::cout << "Joint velocity limits exceeded." << std::endl;
    } else {
        std::cout << "Joint velocity limits satisfied." << std::endl;
    }

    return 0;
}
      
