// Chapter17_Lesson4.cpp
// Noise Modeling in Sensors and Actuators for Dynamic Systems
// C++17 simulation of a mass-spring-damper with actuator and sensor noise.

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

int main() {
    const double m = 1.2;
    const double c = 0.35;
    const double k = 4.0;
    const double Ts = 0.01;
    const int N = 6000;

    // Noise parameters
    const double sigma_w = 0.20;       // process force noise std
    const double sigma_v = 0.01;       // sensor white noise std
    const double sigma_bias_rw = 0.002;// bias random-walk intensity
    const double delta_q = 0.001;      // quantizer step
    const double rho_a = 0.97;         // actuator colored-noise coefficient
    const double sigma_a_ss = 0.08;    // actuator steady-state std

    const double PI = 3.14159265358979323846;

    std::mt19937 rng(17);
    std::normal_distribution<double> n01(0.0, 1.0);

    // States: x1 = position, x2 = velocity
    double x1 = 0.0, x2 = 0.0;
    double bias = 0.0;
    double aNoise = 0.0;

    std::vector<double> y(N, 0.0), yTrue(N, 0.0);

    // Theoretical covariance recursion for x = [x1, x2]^T
    // A = I + Ts * [[0,1],[-k/m,-c/m]], B = Ts * [[0],[1/m]], G = B
    const double A11 = 1.0;
    const double A12 = Ts;
    const double A21 = -Ts * k / m;
    const double A22 = 1.0 - Ts * c / m;
    const double B1 = 0.0;
    const double B2 = Ts / m;
    const double G1 = 0.0;
    const double G2 = Ts / m;

    // P = [[p11,p12],[p12,p22]]
    double p11 = 0.0, p12 = 0.0, p22 = 0.0;
    const double Qw = sigma_w * sigma_w;
    const double Qa = sigma_a_ss * sigma_a_ss;
    const double Rv = sigma_v * sigma_v;
    const double Rq = (delta_q * delta_q) / 12.0;
    std::vector<double> Syy(N, 0.0);

    for (int kIdx = 0; kIdx < N - 1; ++kIdx) {
        double t = kIdx * Ts;
        double uCmd = 0.8 * std::sin(2.0 * PI * 0.7 * t);

        // Actuator colored noise (AR(1))
        aNoise = rho_a * aNoise + std::sqrt(1.0 - rho_a * rho_a) * sigma_a_ss * n01(rng);

        // Process and sensor noise
        double w = sigma_w * n01(rng);
        bias += sigma_bias_rw * std::sqrt(Ts) * n01(rng);
        double v = sigma_v * n01(rng);

        // Plant update
        double uActual = uCmd + aNoise;
        double x1n = A11 * x1 + A12 * x2 + B1 * uActual + G1 * w;
        double x2n = A21 * x1 + A22 * x2 + B2 * uActual + G2 * w;
        x1 = x1n;
        x2 = x2n;

        // Sensor model
        double yAnalog = x1 + bias + v;
        double q = delta_q * std::round(yAnalog / delta_q) - yAnalog;
        double yMeas = yAnalog + q;

        y[kIdx + 1] = yMeas;
        yTrue[kIdx + 1] = x1;

        // Covariance update P <- A P A^T + GQwG^T + BQaB^T
        double ap11 = A11 * p11 + A12 * p12;
        double ap12 = A11 * p12 + A12 * p22;
        double ap21 = A21 * p11 + A22 * p12;
        double ap22 = A21 * p12 + A22 * p22;

        double p11n = ap11 * A11 + ap12 * A12 + G1 * Qw * G1 + B1 * Qa * B1;
        double p12n = ap11 * A21 + ap12 * A22 + G1 * Qw * G2 + B1 * Qa * B2;
        double p22n = ap21 * A21 + ap22 * A22 + G2 * Qw * G2 + B2 * Qa * B2;

        p11 = p11n;
        p12 = p12n;
        p22 = p22n;

        Syy[kIdx + 1] = p11 + Rv + Rq; // C = [1, 0]
    }

    // Empirical variance after burn-in
    const int burn = 1000;
    int M = N - burn;
    double meanY = 0.0, meanYTrue = 0.0, meanSyy = 0.0;
    for (int i = burn; i < N; ++i) {
        meanY += y[i];
        meanYTrue += yTrue[i];
        meanSyy += Syy[i];
    }
    meanY /= M;
    meanYTrue /= M;
    meanSyy /= M;

    double varY = 0.0, varYTrue = 0.0;
    for (int i = burn; i < N; ++i) {
        varY += (y[i] - meanY) * (y[i] - meanY);
        varYTrue += (yTrue[i] - meanYTrue) * (yTrue[i] - meanYTrue);
    }
    varY /= (M - 1);
    varYTrue /= (M - 1);

    std::cout << "=== Noise Modeling Demo (C++) ===\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Empirical var(y_true): " << varYTrue << "\n";
    std::cout << "Empirical var(y)    : " << varY << "\n";
    std::cout << "Predicted mean Syy  : " << meanSyy << " (bias RW excluded)\n";
    std::cout << "Quantization theory : " << (delta_q * delta_q / 12.0) << "\n";

    return 0;
}
