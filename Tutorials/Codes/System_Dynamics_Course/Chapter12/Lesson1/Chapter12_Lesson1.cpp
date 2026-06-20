/*
Chapter12_Lesson1.cpp
System Dynamics (Control Engineering) — Chapter 12, Lesson 1
Sinusoidal Steady-State Response and Frequency Response Definition

This program:
1) Evaluates a 2nd-order transfer function at s = j*omega to obtain G(jw).
2) Simulates the equivalent ODE with RK4 under sinusoidal forcing.
3) Fits the steady-state output to extract amplitude and phase.

Build (example):
  g++ -O2 -std=c++17 Chapter12_Lesson1.cpp -o Chapter12_Lesson1

No external dependencies.
*/

#include <complex>
#include <iostream>
#include <vector>
#include <cmath>

static constexpr double PI = 3.14159265358979323846;

std::complex<double> polyval(const std::vector<double>& c, std::complex<double> s) {
    // c in descending powers
    std::complex<double> y = 0.0;
    for (double a : c) y = y * s + a;
    return y;
}

std::complex<double> tf_eval(const std::vector<double>& num,
                             const std::vector<double>& den,
                             std::complex<double> s) {
    return polyval(num, s) / polyval(den, s);
}

struct FitResult {
    double R;   // amplitude
    double phi; // phase (rad) in y ≈ R sin(omega t + phi)
    double a;   // sin coeff
    double b;   // cos coeff
};

// Least squares fit y ≈ a sin(wt) + b cos(wt)
FitResult fit_sinusoid(const std::vector<double>& t,
                       const std::vector<double>& y,
                       double omega) {
    double s11 = 0.0, s12 = 0.0, s22 = 0.0;
    double r1  = 0.0, r2  = 0.0;

    for (size_t i = 0; i < t.size(); ++i) {
        double si = std::sin(omega * t[i]);
        double ci = std::cos(omega * t[i]);
        s11 += si * si;
        s12 += si * ci;
        s22 += ci * ci;
        r1  += si * y[i];
        r2  += ci * y[i];
    }

    // Solve 2x2 normal equations:
    // [s11 s12; s12 s22] [a; b] = [r1; r2]
    double det = s11 * s22 - s12 * s12;
    double a = ( r1 * s22 - r2 * s12) / det;
    double b = (-r1 * s12 + r2 * s11) / det;

    double R = std::sqrt(a*a + b*b);
    double phi = std::atan2(b, a); // because R sin(wt+phi)=R cos(phi) sin(wt)+R sin(phi) cos(wt)

    return {R, phi, a, b};
}

double wrap_to_pi(double x) {
    while (x >  PI) x -= 2.0*PI;
    while (x < -PI) x += 2.0*PI;
    return x;
}

int main() {
    // Second-order low-pass: G(s)=wn^2/(s^2+2*zeta*wn*s+wn^2)
    const double wn   = 5.0;
    const double zeta = 0.2;

    std::vector<double> num{wn*wn};
    std::vector<double> den{1.0, 2.0*zeta*wn, wn*wn};

    // Input: u(t) = Um sin(omega t + phi_u)
    const double Um   = 1.0;
    const double omega = 4.0;
    const double phi_u = 0.0;

    std::complex<double> Gjw = tf_eval(num, den, std::complex<double>(0.0, omega));
    double Ym_pred = std::abs(Gjw) * Um;
    double phi_y_pred = std::arg(Gjw) + phi_u;

    std::cout << "G(jw) = " << Gjw << "\n";
    std::cout << "|G(jw)| = " << std::abs(Gjw) << "  angle(G(jw)) = " << std::arg(Gjw) << "\n";
    std::cout << "Predicted steady-state amplitude Ym = " << Ym_pred << "\n";
    std::cout << "Predicted steady-state phase phi_y (rad) = " << phi_y_pred << "\n\n";

    // ODE: y'' + 2*zeta*wn*y' + wn^2*y = wn^2*u(t)
    // State: x1=y, x2=y'
    auto f = [&](double t, double x1, double x2) {
        double u = Um * std::sin(omega*t + phi_u);
        double dx1 = x2;
        double dx2 = -2.0*zeta*wn*x2 - wn*wn*x1 + wn*wn*u;
        return std::pair<double,double>(dx1, dx2);
    };

    // RK4 simulation
    double t0 = 0.0, tf = 40.0;
    double dt = 0.001;
    size_t N = static_cast<size_t>((tf - t0)/dt) + 1;

    std::vector<double> tvec;
    std::vector<double> yvec;
    tvec.reserve(N);
    yvec.reserve(N);

    double x1 = 0.0, x2 = 0.0;
    double t = t0;
    for (size_t k = 0; k < N; ++k) {
        tvec.push_back(t);
        yvec.push_back(x1);

        auto [k1_1, k1_2] = f(t, x1, x2);
        auto [k2_1, k2_2] = f(t + 0.5*dt, x1 + 0.5*dt*k1_1, x2 + 0.5*dt*k1_2);
        auto [k3_1, k3_2] = f(t + 0.5*dt, x1 + 0.5*dt*k2_1, x2 + 0.5*dt*k2_2);
        auto [k4_1, k4_2] = f(t + dt, x1 + dt*k3_1, x2 + dt*k3_2);

        x1 += (dt/6.0) * (k1_1 + 2.0*k2_1 + 2.0*k3_1 + k4_1);
        x2 += (dt/6.0) * (k1_2 + 2.0*k2_2 + 2.0*k3_2 + k4_2);

        t += dt;
    }

    // Use last 10 seconds for steady-state fitting
    std::vector<double> tss, yss;
    for (size_t i = 0; i < tvec.size(); ++i) {
        if (tvec[i] >= tf - 10.0) {
            tss.push_back(tvec[i]);
            yss.push_back(yvec[i]);
        }
    }

    FitResult fr = fit_sinusoid(tss, yss, omega);

    std::cout << "Estimated from simulation (last 10 s):\n";
    std::cout << "Ym_hat = " << fr.R << "\n";
    std::cout << "phi_y_hat (rad) = " << wrap_to_pi(fr.phi) << "\n\n";

    std::cout << "Errors:\n";
    std::cout << "Amplitude error: " << (fr.R - Ym_pred) << "\n";
    std::cout << "Phase error (rad): " << wrap_to_pi(fr.phi - phi_y_pred) << "\n";

    return 0;
}
