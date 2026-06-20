// Chapter18_Lesson1.cpp
// Work, Energy, and Power in Mechanical, Electrical, Fluid, and Thermal Systems
// Standard C++17, no external dependencies.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

static double trapz(const std::vector<double>& y, const std::vector<double>& t) {
    double s = 0.0;
    for (std::size_t k = 0; k + 1 < y.size(); ++k) {
        s += 0.5 * (y[k] + y[k + 1]) * (t[k + 1] - t[k]);
    }
    return s;
}

int main() {
    const double t0 = 0.0, tf = 12.0, dt = 1e-4;
    const int N = static_cast<int>((tf - t0) / dt) + 1;

    std::vector<double> t(N);
    for (int k = 0; k < N; ++k) t[k] = t0 + k * dt;

    // ==========================================================
    // 1) Mechanical: m xdd + c xd + k x = F(t)
    // ==========================================================
    const double m = 1.5, c = 0.8, ks = 12.0;
    auto Fin = [](double tt) { return 2.5 * std::sin(1.2 * tt) + 1.2 * std::cos(0.4 * tt); };

    std::vector<double> x(N, 0.0), v(N, 0.0), Pm_in(N, 0.0), Pm_diss(N, 0.0), Em(N, 0.0);

    for (int k = 0; k < N - 1; ++k) {
        const double f = Fin(t[k]);
        const double a = (f - c * v[k] - ks * x[k]) / m;
        v[k + 1] = v[k] + dt * a;
        x[k + 1] = x[k] + dt * v[k];

        Pm_in[k] = f * v[k];
        Pm_diss[k] = c * v[k] * v[k];
        Em[k] = 0.5 * m * v[k] * v[k] + 0.5 * ks * x[k] * x[k];
    }
    Pm_in[N - 1] = Fin(t[N - 1]) * v[N - 1];
    Pm_diss[N - 1] = c * v[N - 1] * v[N - 1];
    Em[N - 1] = 0.5 * m * v[N - 1] * v[N - 1] + 0.5 * ks * x[N - 1] * x[N - 1];

    const double mechResidual = Em.back() - Em.front() - (trapz(Pm_in, t) - trapz(Pm_diss, t));

    // ==========================================================
    // 2) Electrical: series RLC
    //    L didt + R i + q/C = v_in(t), dq/dt = i
    // ==========================================================
    const double R = 2.0, L = 0.6, C = 0.25;
    auto Vin = [](double tt) { return 5.0 * std::sin(2.0 * tt) + 2.0 * std::cos(0.5 * tt); };

    std::vector<double> q(N, 0.0), i(N, 0.0), Pe_in(N, 0.0), Pe_diss(N, 0.0), Ee(N, 0.0);

    for (int k = 0; k < N - 1; ++k) {
        const double vin = Vin(t[k]);
        const double di = (vin - R * i[k] - q[k] / C) / L;
        const double dq = i[k];

        i[k + 1] = i[k] + dt * di;
        q[k + 1] = q[k] + dt * dq;

        Pe_in[k] = vin * i[k];
        Pe_diss[k] = R * i[k] * i[k];
        Ee[k] = 0.5 * L * i[k] * i[k] + 0.5 * q[k] * q[k] / C;
    }
    Pe_in[N - 1] = Vin(t[N - 1]) * i[N - 1];
    Pe_diss[N - 1] = R * i[N - 1] * i[N - 1];
    Ee[N - 1] = 0.5 * L * i[N - 1] * i[N - 1] + 0.5 * q[N - 1] * q[N - 1] / C;

    const double elecResidual = Ee.back() - Ee.front() - (trapz(Pe_in, t) - trapz(Pe_diss, t));

    // ==========================================================
    // 3) Fluid: C_h dp/dt = q_in - q,   L_h dq/dt = p - R_h q
    // ==========================================================
    const double Ch = 0.08, Lh = 0.15, Rh = 1.7;
    auto qin = [](double tt) { return 0.8 * std::sin(0.9 * tt) + 0.2 * std::cos(0.3 * tt); };

    std::vector<double> p(N, 0.0), qf(N, 0.0), Pf_in(N, 0.0), Pf_diss(N, 0.0), Ef(N, 0.0);

    for (int k = 0; k < N - 1; ++k) {
        const double qsrc = qin(t[k]);
        const double dp = (qsrc - qf[k]) / Ch;
        const double dqf = (p[k] - Rh * qf[k]) / Lh;

        p[k + 1] = p[k] + dt * dp;
        qf[k + 1] = qf[k] + dt * dqf;

        Pf_in[k] = p[k] * qsrc;
        Pf_diss[k] = Rh * qf[k] * qf[k];
        Ef[k] = 0.5 * Ch * p[k] * p[k] + 0.5 * Lh * qf[k] * qf[k];
    }
    Pf_in[N - 1] = p[N - 1] * qin(t[N - 1]);
    Pf_diss[N - 1] = Rh * qf[N - 1] * qf[N - 1];
    Ef[N - 1] = 0.5 * Ch * p[N - 1] * p[N - 1] + 0.5 * Lh * qf[N - 1] * qf[N - 1];

    const double fluidResidual = Ef.back() - Ef.front() - (trapz(Pf_in, t) - trapz(Pf_diss, t));

    // ==========================================================
    // 4) Thermal: C_th dT/dt = Q_in - (T - T_env)/R_th
    // ==========================================================
    const double Cth = 600.0, Rth = 0.4, Tenv = 293.15;
    auto Qin = [](double tt) { return 120.0 + 40.0 * std::sin(0.15 * tt); };

    std::vector<double> T(N, Tenv), Pt_in(N, 0.0), Pt_out(N, 0.0), Et(N, 0.0);

    for (int k = 0; k < N - 1; ++k) {
        const double qdotIn = Qin(t[k]);
        const double qdotOut = (T[k] - Tenv) / Rth;
        const double dT = (qdotIn - qdotOut) / Cth;

        T[k + 1] = T[k] + dt * dT;

        Pt_in[k] = qdotIn;
        Pt_out[k] = qdotOut;
        Et[k] = Cth * (T[k] - Tenv);
    }
    Pt_in[N - 1] = Qin(t[N - 1]);
    Pt_out[N - 1] = (T[N - 1] - Tenv) / Rth;
    Et[N - 1] = Cth * (T[N - 1] - Tenv);

    const double thermResidual = Et.back() - Et.front() - (trapz(Pt_in, t) - trapz(Pt_out, t));

    std::cout << std::scientific << std::setprecision(6);
    std::cout << "=== Energy Balance Audit (C++) ===\n";
    std::cout << "Mechanical residual : " << mechResidual << "\n";
    std::cout << "Electrical residual : " << elecResidual << "\n";
    std::cout << "Fluid residual      : " << fluidResidual << "\n";
    std::cout << "Thermal residual    : " << thermResidual << "\n\n";

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Final stored energies:\n";
    std::cout << "E_mech(tf)  = " << Em.back() << "\n";
    std::cout << "E_elec(tf)  = " << Ee.back() << "\n";
    std::cout << "E_fluid(tf) = " << Ef.back() << "\n";
    std::cout << "E_therm(tf) = " << Et.back() << "\n";
    return 0;
}
