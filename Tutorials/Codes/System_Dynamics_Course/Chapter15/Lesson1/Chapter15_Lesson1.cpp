// Chapter15_Lesson1.cpp
// Euler and Improved Euler (Heun) for y' = -2 y + sin(t), y(0)=1

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

double f(double t, double y) {
    return -2.0 * y + std::sin(t);
}

double exact_solution(double t) {
    return (2.0 * std::sin(t) - std::cos(t)) / 5.0 + (6.0 / 5.0) * std::exp(-2.0 * t);
}

void euler(double t0, double tf, double y0, double h,
           std::vector<double>& t, std::vector<double>& y) {
    int n = static_cast<int>(std::round((tf - t0) / h));
    t.resize(n + 1);
    y.resize(n + 1);
    t[0] = t0;
    y[0] = y0;
    for (int k = 0; k < n; ++k) {
        t[k + 1] = t[k] + h;
        y[k + 1] = y[k] + h * f(t[k], y[k]);
    }
}

void improved_euler(double t0, double tf, double y0, double h,
                    std::vector<double>& t, std::vector<double>& y) {
    int n = static_cast<int>(std::round((tf - t0) / h));
    t.resize(n + 1);
    y.resize(n + 1);
    t[0] = t0;
    y[0] = y0;
    for (int k = 0; k < n; ++k) {
        t[k + 1] = t[k] + h;
        double s1 = f(t[k], y[k]);
        double ypred = y[k] + h * s1;
        double s2 = f(t[k + 1], ypred);
        y[k + 1] = y[k] + 0.5 * h * (s1 + s2);
    }
}

double max_abs_error(const std::vector<double>& t, const std::vector<double>& y) {
    double emax = 0.0;
    for (std::size_t k = 0; k < t.size(); ++k) {
        double e = std::abs(y[k] - exact_solution(t[k]));
        if (e > emax) {
            emax = e;
        }
    }
    return emax;
}

double estimate_order(double err_h, double err_h2) {
    return std::log(err_h / err_h2) / std::log(2.0);
}

int main() {
    const double t0 = 0.0;
    const double tf = 5.0;
    const double y0 = 1.0;
    std::vector<double> hs = {0.2, 0.1, 0.05, 0.025};

    std::vector<double> eErr, hErr;

    std::cout << "Step-size study (max error on [0,5])\n";
    std::cout << std::left << std::setw(10) << "h"
              << std::setw(18) << "Euler"
              << std::setw(18) << "ImprovedEuler" << "\n";

    for (double h : hs) {
        std::vector<double> tE, yE, tH, yH;
        euler(t0, tf, y0, h, tE, yE);
        improved_euler(t0, tf, y0, h, tH, yH);
        double ee = max_abs_error(tE, yE);
        double he = max_abs_error(tH, yH);
        eErr.push_back(ee);
        hErr.push_back(he);

        std::cout << std::left << std::setw(10) << h
                  << std::setw(18) << ee
                  << std::setw(18) << he << "\n";
    }

    std::cout << "\nEstimated order (successive halving)\n";
    for (std::size_t i = 0; i + 1 < hs.size(); ++i) {
        std::cout << "h=" << hs[i] << " -> " << hs[i + 1]
                  << " : Euler p~" << estimate_order(eErr[i], eErr[i + 1])
                  << ", Heun p~" << estimate_order(hErr[i], hErr[i + 1]) << "\n";
    }

    // Save representative trajectory to CSV
    std::vector<double> t, y;
    improved_euler(t0, tf, y0, 0.1, t, y);
    std::ofstream out("Chapter15_Lesson1_cpp_output.csv");
    out << "t,heun,exact,abs_error\n";
    for (std::size_t k = 0; k < t.size(); ++k) {
        double ex = exact_solution(t[k]);
        out << t[k] << "," << y[k] << "," << ex << "," << std::abs(y[k] - ex) << "\n";
    }
    out.close();

    return 0;
}
