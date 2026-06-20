// Chapter18_Lesson4.cpp
// Bond graph simulation: Se - 1 - (I,R,C) for a mass-spring-damper
// Compile: g++ -std=c++17 Chapter18_Lesson4.cpp -o Chapter18_Lesson4

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

struct Params { double m, b, C; };

double Se(double t) {
    return 2.0 * std::sin(2.0 * M_PI * 0.8 * t) + (t >= 1.0 ? 1.0 : 0.0);
}

std::array<double,2> rhs(double t, const std::array<double,2>& x, const Params& p) {
    double q = x[0];
    double mom = x[1];
    double v = mom / p.m;
    double eR = p.b * v;
    double eC = q / p.C;
    return {v, Se(t) - eR - eC};
}

std::array<double,2> rk4(double t, const std::array<double,2>& x, double h, const Params& p) {
    auto k1 = rhs(t, x, p);
    auto x2 = std::array<double,2>{x[0] + 0.5*h*k1[0], x[1] + 0.5*h*k1[1]};
    auto k2 = rhs(t + 0.5*h, x2, p);
    auto x3 = std::array<double,2>{x[0] + 0.5*h*k2[0], x[1] + 0.5*h*k2[1]};
    auto k3 = rhs(t + 0.5*h, x3, p);
    auto x4 = std::array<double,2>{x[0] + h*k3[0], x[1] + h*k3[1]};
    auto k4 = rhs(t + h, x4, p);

    return {
        x[0] + (h/6.0)*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0]),
        x[1] + (h/6.0)*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
    };
}

int main() {
    Params p{1.5, 1.2, 1.0/12.0};
    double h = 1e-3, t0 = 0.0, tf = 10.0;
    int N = static_cast<int>((tf - t0)/h) + 1;

    std::vector<double> t(N), q(N), mom(N), v(N), H(N), Ediss(N), res(N);
    q[0] = 0.10; mom[0] = 0.0; Ediss[0] = 0.0;
    for (int i = 0; i < N; ++i) t[i] = t0 + i*h;

    for (int i = 0; i < N - 1; ++i) {
        auto xn = rk4(t[i], {q[i], mom[i]}, h, p);
        q[i+1] = xn[0];
        mom[i+1] = xn[1];
    }

    for (int i = 0; i < N; ++i) {
        v[i] = mom[i]/p.m;
        H[i] = 0.5*mom[i]*mom[i]/p.m + 0.5*q[i]*q[i]/p.C;
    }

    for (int i = 1; i < N; ++i) {
        Ediss[i] = Ediss[i-1] + h * (p.b * v[i-1] * v[i-1]);
    }

    double maxRes = 0.0;
    for (int i = 1; i < N - 1; ++i) {
        double dH = (H[i+1] - H[i-1])/(2*h);
        res[i] = Se(t[i])*v[i] - p.b*v[i]*v[i] - dH;
        if (std::abs(res[i]) > maxRes) maxRes = std::abs(res[i]);
    }
    std::cout << "Max |power residual| = " << maxRes << "\n";

    std::ofstream out("Chapter18_Lesson4_cpp_output.csv");
    out << "t,q,v,H,Ediss,residual\n";
    out << std::setprecision(10);
    for (int i = 0; i < N; ++i) {
        out << t[i] << "," << q[i] << "," << v[i] << "," << H[i] << "," << Ediss[i] << "," << res[i] << "\n";
    }
    out.close();
    return 0;
}
