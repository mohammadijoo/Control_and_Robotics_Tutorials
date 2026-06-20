// Chapter5_Lesson3.cpp
// Autonomous Mobile Robots (Control Engineering) — Chapter 5, Lesson 3
// Topic: Drift Sources and Bias Accumulation
//
// Build (example):
//   g++ -O2 -std=c++17 Chapter5_Lesson3.cpp -o ch5_l3
//
// This program simulates differential-drive dead-reckoning with parameter bias,
// encoder quantization, and gyro bias, and prints final drift metrics.

#include <iostream>
#include <vector>
#include <cmath>
#include <random>

static double wrap_pi(double a) {
    const double pi = 3.14159265358979323846;
    a = std::fmod(a + pi, 2.0*pi);
    if (a < 0) a += 2.0*pi;
    return a - pi;
}

struct Result {
    double final_pos_err_wheel;
    double final_pos_err_gyro;
    double final_head_err_wheel_deg;
    double final_head_err_gyro_deg;
};

Result simulate(
    double T=60.0, double dt=0.01,
    double r_true=0.05, double b_true=0.30, int ticks_per_rev=2048,
    double r_hat=0.05*1.005, double b_hat=0.30*0.995,
    double eps_r_L=+0.002, double eps_r_R=-0.002,
    double encoder_tick_noise_std=0.2,
    double gyro_bias=0.005, double gyro_noise_std=0.002,
    unsigned seed=7
) {
    int N = (int)std::floor(T/dt);
    std::mt19937 gen(seed);
    std::normal_distribution<double> n01(0.0, 1.0);

    const double pi = 3.14159265358979323846;
    const double rad_per_tick = 2.0*pi / (double)ticks_per_rev;

    double rL_true = r_true*(1.0 + eps_r_L);
    double rR_true = r_true*(1.0 + eps_r_R);

    double x=0, y=0, th=0;
    double xw=0, yw=0, thw=0;
    double xg=0, yg=0, thg=0;
    double th_gyro=0;

    for (int k=1; k<N; ++k) {
        double t = (k-1)*dt;
        double v = 0.6;
        double w = 0.10*std::sin(2.0*pi*t/20.0);

        // True wheel angular velocities
        double wR = (v + 0.5*b_true*w)/rR_true;
        double wL = (v - 0.5*b_true*w)/rL_true;

        double dphiR_true = wR*dt;
        double dphiL_true = wL*dt;

        // Quantize to ticks + noise
        double ticksR = dphiR_true / rad_per_tick;
        double ticksL = dphiL_true / rad_per_tick;

        double ticksR_meas = std::round(ticksR) + encoder_tick_noise_std*n01(gen);
        double ticksL_meas = std::round(ticksL) + encoder_tick_noise_std*n01(gen);

        double dphiR_meas = ticksR_meas * rad_per_tick;
        double dphiL_meas = ticksL_meas * rad_per_tick;

        // Ground truth update
        double dSR_true = rR_true*dphiR_true;
        double dSL_true = rL_true*dphiL_true;
        double dS_true  = 0.5*(dSR_true + dSL_true);
        double dTh_true = (dSR_true - dSL_true)/b_true;

        th = wrap_pi(th + dTh_true);
        x  = x + dS_true*std::cos(th - 0.5*dTh_true);
        y  = y + dS_true*std::sin(th - 0.5*dTh_true);

        // Wheel-only estimate
        double dSR_hat = r_hat*dphiR_meas;
        double dSL_hat = r_hat*dphiL_meas;
        double dS_hat  = 0.5*(dSR_hat + dSL_hat);
        double dTh_hat = (dSR_hat - dSL_hat)/b_hat;

        thw = wrap_pi(thw + dTh_hat);
        xw  = xw + dS_hat*std::cos(thw - 0.5*dTh_hat);
        yw  = yw + dS_hat*std::sin(thw - 0.5*dTh_hat);

        // Gyro heading
        double w_meas = w + gyro_bias + gyro_noise_std*n01(gen);
        th_gyro = wrap_pi(th_gyro + w_meas*dt);
        thg = th_gyro;
        xg = xg + dS_hat*std::cos(thg);
        yg = yg + dS_hat*std::sin(thg);
    }

    double e_w = std::sqrt((xw-x)*(xw-x) + (yw-y)*(yw-y));
    double e_g = std::sqrt((xg-x)*(xg-x) + (yg-y)*(yg-y));
    double he_w = (180.0/pi)*wrap_pi(thw-th);
    double he_g = (180.0/pi)*wrap_pi(thg-th);

    return {e_w, e_g, he_w, he_g};
}

int main() {
    Result r = simulate();
    std::cout << "Final position error (wheel-only) [m]: " << r.final_pos_err_wheel << "\n";
    std::cout << "Final position error (gyro-heading) [m]: " << r.final_pos_err_gyro << "\n";
    std::cout << "Final heading error (wheel-only) [deg]: " << r.final_head_err_wheel_deg << "\n";
    std::cout << "Final heading error (gyro-heading) [deg]: " << r.final_head_err_gyro_deg << "\n";
    return 0;
}
