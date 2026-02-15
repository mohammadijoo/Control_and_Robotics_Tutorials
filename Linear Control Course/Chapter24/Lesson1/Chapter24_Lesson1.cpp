#include <iostream>
#include <vector>
#include <complex>
#include <cmath>

using std::complex;
using std::vector;

complex<double> eval_tf(const vector<double>& num,
                         const vector<double>& den,
                         double w)
{
    complex<double> s(0.0, w);
    complex<double> N(0.0, 0.0), D(0.0, 0.0);

    // Horner evaluation
    for (std::size_t i = 0; i < num.size(); ++i)
    {
        N = N * s + num[i];
    }
    for (std::size_t i = 0; i < den.size(); ++i)
    {
        D = D * s + den[i];
    }
    return N / D;
}

int main()
{
    // Robot joint plant G(s) = K_t / (J s^2 + b s)
    double J  = 0.01;
    double b  = 0.1;
    double Kt = 0.5;

    // Numerator and denominator for G(s)
    vector<double> G_num{Kt};
    vector<double> G_den{J, b, 0.0};

    // PI controller C(s) = (Kp s + Ki) / s
    double Kp = 20.0;
    double Ki = 10.0;
    vector<double> C_num{Kp, Ki};
    vector<double> C_den{1.0, 0.0};

    // Frequency grid (rad/s)
    double w_min = 0.1;
    double w_max = 100.0;
    int N = 2000;
    double log_w_min = std::log10(w_min);
    double log_w_max = std::log10(w_max);

    double best_gc_diff = 1e9;
    double best_gc = 0.0;
    double best_pc_diff = 1e9;
    double best_pc = 0.0;
    double phase_at_gc = 0.0;
    double mag_at_pc = 0.0;

    for (int k = 0; k < N; ++k)
    {
        double logw = log_w_min + (log_w_max - log_w_min) * k / (N - 1);
        double w = std::pow(10.0, logw);
        complex<double> s(0.0, w);

        complex<double> G = eval_tf(G_num, G_den, w);
        complex<double> C = eval_tf(C_num, C_den, w);
        complex<double> L = C * G;

        double mag = std::abs(L);
        double phase = std::arg(L); // radians

        // Track approximate gain crossover |L| = 1
        double gc_diff = std::fabs(mag - 1.0);
        if (gc_diff < best_gc_diff)
        {
            best_gc_diff = gc_diff;
            best_gc = w;
            phase_at_gc = phase;
        }

        // Track approximate phase crossover angle = -pi
        double pc_diff = std::fabs(phase + M_PI);
        if (pc_diff < best_pc_diff)
        {
            best_pc_diff = pc_diff;
            best_pc = w;
            mag_at_pc = mag;
        }
    }

    // Phase margin (deg)
    double pm = (M_PI + phase_at_gc) * 180.0 / M_PI;

    // Gain margin factor
    double gm = (mag_at_pc > 0.0) ? (1.0 / mag_at_pc) : INFINITY;
    double gm_db = 20.0 * std::log10(gm);

    // Delay margin approximation
    double pm_rad = pm * M_PI / 180.0;
    double tau_max = pm_rad / best_gc;

    std::cout << "Approx gain crossover omega_gc = " << best_gc << " rad/s\n";
    std::cout << "Approx phase crossover omega_pc = " << best_pc << " rad/s\n";
    std::cout << "Phase margin pm = " << pm << " deg\n";
    std::cout << "Gain margin gm = " << gm << " (" << gm_db << " dB)\n";
    std::cout << "Approx delay margin tau_max = " << tau_max << " s\n";

    // In a ROS2-based robot, the resulting tau_max can be compared against
    // control loop sampling, communication, and computation delays.

    return 0;
}
