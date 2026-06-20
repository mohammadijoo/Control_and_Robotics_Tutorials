// Chapter14_Lesson5.cpp
// Limit Cycles, Multiple Equilibria, and Basic Bifurcation Notions
// C++17: RK4 simulation of Van der Pol oscillator and equilibrium classification
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <string>

struct State {
    double x;
    double y;
};

State vdp_rhs(const State& s, double mu) {
    State ds;
    ds.x = s.y;
    ds.y = mu * (1.0 - s.x * s.x) * s.y - s.x;
    return ds;
}

State add(const State& a, const State& b, double scale=1.0) {
    return {a.x + scale * b.x, a.y + scale * b.y};
}

State rk4_step(const State& s, double h, double mu) {
    State k1 = vdp_rhs(s, mu);
    State k2 = vdp_rhs(add(s, k1, 0.5*h), mu);
    State k3 = vdp_rhs(add(s, k2, 0.5*h), mu);
    State k4 = vdp_rhs(add(s, k3, h), mu);
    State out;
    out.x = s.x + (h/6.0) * (k1.x + 2.0*k2.x + 2.0*k3.x + k4.x);
    out.y = s.y + (h/6.0) * (k1.y + 2.0*k2.y + 2.0*k3.y + k4.y);
    return out;
}

double estimate_period(const std::vector<double>& t, const std::vector<State>& z) {
    std::vector<double> crossings;
    for (size_t k = 0; k + 1 < z.size(); ++k) {
        if (z[k].y < 0.0 && z[k+1].y >= 0.0) {
            double denom = (z[k+1].y - z[k].y);
            if (std::abs(denom) < 1e-14) continue;
            double alpha = -z[k].y / denom;
            double tc = t[k] + alpha * (t[k+1] - t[k]);
            double xc = z[k].x + alpha * (z[k+1].x - z[k].x);
            if (xc > 0.0) crossings.push_back(tc);
        }
    }
    if (crossings.size() < 3) return std::nan("");
    size_t start = (crossings.size() > 5) ? crossings.size() - 5 : 1;
    double sum = 0.0;
    int cnt = 0;
    for (size_t i = start; i < crossings.size(); ++i) {
        sum += (crossings[i] - crossings[i-1]);
        cnt++;
    }
    return (cnt > 0) ? sum / cnt : std::nan("");
}

std::string stability_pitchfork(double r, double xeq) {
    double lambda = r - 3.0 * xeq * xeq;
    if (lambda < 0.0) return "stable";
    if (lambda > 0.0) return "unstable";
    return "nonhyperbolic";
}

std::string stability_saddlenode(double xeq) {
    double lambda = -2.0 * xeq;
    if (lambda < 0.0) return "stable";
    if (lambda > 0.0) return "unstable";
    return "nonhyperbolic";
}

int main() {
    const double mu = 1.0;
    const double h = 0.01;
    const double T = 80.0;
    const int N = static_cast<int>(T / h);

    std::vector<double> t(N + 1);
    std::vector<State> z(N + 1);
    z[0] = {2.0, 0.1};
    t[0] = 0.0;

    for (int k = 0; k < N; ++k) {
        t[k+1] = t[k] + h;
        z[k+1] = rk4_step(z[k], h, mu);
    }

    // Write phase-portrait data to CSV for plotting externally
    std::ofstream csv("Chapter14_Lesson5_vdp_phase.csv");
    csv << "t,x,y\n";
    for (int k = 0; k <= N; ++k) {
        csv << t[k] << "," << z[k].x << "," << z[k].y << "\n";
    }
    csv.close();

    // Estimate asymptotic period from latter part
    std::vector<double> t_tail(t.begin() + N/2, t.end());
    std::vector<State> z_tail(z.begin() + N/2, z.end());
    double period = estimate_period(t_tail, z_tail);

    double amp = 0.0;
    for (size_t k = N/2; k < z.size(); ++k) amp = std::max(amp, std::abs(z[k].x));

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Van der Pol (mu=" << mu << ") limit-cycle estimate\n";
    std::cout << "Amplitude |x|_max ~= " << amp << "\n";
    std::cout << "Period ~= " << period << "\n\n";

    // Print equilibrium branches for two normal forms (sampled values)
    std::cout << "Pitchfork normal form: xdot = r*x - x^3\n";
    for (double r : {-1.0, 0.0, 1.0}) {
        std::cout << "r = " << r << ": ";
        std::cout << "x*=0 (" << stability_pitchfork(r, 0.0) << ")";
        if (r >= 0.0) {
            double x1 = std::sqrt(r), x2 = -std::sqrt(r);
            std::cout << ", x*=+" << x1 << " (" << stability_pitchfork(r, x1) << ")";
            std::cout << ", x*=" << x2 << " (" << stability_pitchfork(r, x2) << ")";
        }
        std::cout << "\n";
    }

    std::cout << "\nSaddle-node normal form: xdot = r - x^2\n";
    for (double r : {-1.0, 0.0, 1.0}) {
        std::cout << "r = " << r << ": ";
        if (r < 0.0) {
            std::cout << "no real equilibria\n";
        } else if (r == 0.0) {
            std::cout << "x*=0 (nonhyperbolic)\n";
        } else {
            double x1 = std::sqrt(r), x2 = -std::sqrt(r);
            std::cout << "x*=+" << x1 << " (" << stability_saddlenode(x1) << "), ";
            std::cout << "x*=" << x2 << " (" << stability_saddlenode(x2) << ")\n";
        }
    }

    std::cout << "\nCSV written: Chapter14_Lesson5_vdp_phase.csv\n";
    return 0;
}
