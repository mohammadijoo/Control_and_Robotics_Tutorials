// Chapter19_Lesson1.cpp
// 1D heat/diffusion and wave equations (explicit finite differences)
// Compile: g++ -O2 -std=c++17 Chapter19_Lesson1.cpp -o ch19_l1

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

int main() {
    // Heat / diffusion equation: u_t = alpha u_xx
    const double L = 1.0, alpha = 0.2;
    const int Nx = 81;
    const double dx = L / (Nx - 1);
    const double r = 0.45;                 // r <= 0.5
    const double dt = r * dx * dx / alpha;
    const int Nt = static_cast<int>(0.25 / dt);

    std::vector<double> x(Nx), u(Nx), un(Nx);
    for (int i = 0; i < Nx; ++i) {
        x[i] = i * dx;
        u[i] = std::sin(M_PI * x[i]) + 0.2 * std::sin(3.0 * M_PI * x[i]);
    }
    u.front() = 0.0; u.back() = 0.0;

    std::ofstream heatFile("Chapter19_Lesson1_heat.csv");
    heatFile << "step,x,u\n";
    for (int n = 0; n <= Nt; ++n) {
        for (int i = 0; i < Nx; ++i) { heatFile << n << "," << x[i] << "," << u[i] << "\n"; }
        un = u;
        for (int i = 1; i < Nx - 1; ++i) {
            u[i] = un[i] + alpha * dt / (dx * dx) * (un[i + 1] - 2.0 * un[i] + un[i - 1]);
        }
        u.front() = 0.0; u.back() = 0.0;
    }

    // Wave equation: u_tt = c^2 u_xx
    const double c = 1.0;
    const int Nxw = 201;
    const double dxw = L / (Nxw - 1);
    const double s = 0.95;                 // s <= 1
    const double dtw = s * dxw / c;
    const int Ntw = static_cast<int>(1.0 / dtw);

    std::vector<double> xw(Nxw), up(Nxw), uc(Nxw), unext(Nxw, 0.0);
    for (int i = 0; i < Nxw; ++i) {
        xw[i] = i * dxw;
        up[i] = std::exp(-180.0 * (xw[i] - 0.35) * (xw[i] - 0.35));
    }
    up.front() = 0.0; up.back() = 0.0;

    uc = up;
    double cfl2 = (c * dtw / dxw) * (c * dtw / dxw);
    for (int i = 1; i < Nxw - 1; ++i) {
        uc[i] = up[i] + 0.5 * cfl2 * (up[i + 1] - 2.0 * up[i] + up[i - 1]);
    }
    uc.front() = 0.0; uc.back() = 0.0;

    std::ofstream waveFile("Chapter19_Lesson1_wave.csv");
    waveFile << "step,x,u\n";
    for (int n = 0; n <= Ntw; ++n) {
        for (int i = 0; i < Nxw; ++i) { waveFile << n << "," << xw[i] << "," << uc[i] << "\n"; }
        for (int i = 1; i < Nxw - 1; ++i) {
            unext[i] = 2.0 * uc[i] - up[i] + cfl2 * (uc[i + 1] - 2.0 * uc[i] + uc[i - 1]);
        }
        unext.front() = 0.0; unext.back() = 0.0;
        up = uc;
        uc = unext;
    }

    std::cout << "Saved Chapter19_Lesson1_heat.csv and Chapter19_Lesson1_wave.csv\n";
    return 0;
}
