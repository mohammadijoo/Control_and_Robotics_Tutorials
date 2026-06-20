/*
Chapter 12 - Lesson 5: Time–Frequency Domain Relationships and Trade-offs
File: Chapter12_Lesson5.cpp

Self-contained C++17 program that computes:
  - Resonant peak Mr and -3dB bandwidth wb for the 2nd-order low-pass:
      G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
  - Step-response metrics using the closed-form underdamped response (0 < zeta < 1)

Build:
  g++ -std=c++17 -O2 Chapter12_Lesson5.cpp -o Chapter12_Lesson5

Run:
  ./Chapter12_Lesson5
*/

#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <iomanip>
#include <limits>

static inline double sqr(double x){ return x*x; }

struct FreqMetrics {
    double Mr;
    double wr;
    double wb;
};

std::complex<double> G_jw(double w, double zeta, double wn){
    // Evaluate G(jw) = wn^2 / ((jw)^2 + 2 zeta wn (jw) + wn^2)
    std::complex<double> jw(0.0, w);
    std::complex<double> den = jw*jw + 2.0*zeta*wn*jw + sqr(wn);
    return sqr(wn) / den;
}

FreqMetrics frequency_metrics(double zeta, double wn){
    const int N = 6000;
    double wmin = 1e-2 * wn;
    double wmax = 1e+3 * wn;

    double Mr = 0.0;
    double wr = wmin;

    // DC gain is 1; use a very small w to approximate.
    double dc = std::abs(G_jw(wmin, zeta, wn));
    double target = dc / std::sqrt(2.0);

    double wb = std::numeric_limits<double>::quiet_NaN();
    bool found_bw = false;

    for(int i=0;i<N;i++){
        double a = static_cast<double>(i)/(N-1);
        double w = wmin * std::pow(wmax/wmin, a);
        double mag = std::abs(G_jw(w, zeta, wn));

        if(mag > Mr){
            Mr = mag;
            wr = w;
        }
        if(!found_bw && mag <= target){
            wb = w;
            found_bw = true;
        }
    }
    return {Mr, wr, wb};
}

struct StepMetrics {
    double Mp_percent;
    double tp;
    double ts_2pct;
};

StepMetrics step_metrics_underdamped(double zeta, double wn){
    // Under-damped (0 < zeta < 1) closed-form metrics:
    // Mp = exp(-zeta*pi/sqrt(1-zeta^2))*100%
    // tp = pi/(wn*sqrt(1-zeta^2))
    // ts(2%) ~ 4/(zeta*wn)
    double wd = wn*std::sqrt(1.0 - zeta*zeta);
    double Mp = std::exp(-zeta*M_PI/std::sqrt(1.0 - zeta*zeta))*100.0;
    double tp = M_PI / wd;
    double ts = 4.0 / (zeta*wn);
    return {Mp, tp, ts};
}

int main(){
    double zeta = 0.35;
    double wn   = 12.0;

    auto fm = frequency_metrics(zeta, wn);
    std::cout << "=== C++ demo: 2nd-order time-frequency tradeoffs ===\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "zeta=" << zeta << ", wn=" << wn << " rad/s\n";
    std::cout << "Resonant peak Mr = " << fm.Mr << "\n";
    std::cout << "Resonant freq  wr = " << fm.wr << " rad/s\n";
    std::cout << "Bandwidth (-3dB) wb = " << fm.wb << " rad/s\n";

    if(zeta > 0.0 && zeta < 1.0){
        auto sm = step_metrics_underdamped(zeta, wn);
        std::cout << "Overshoot Mp = " << sm.Mp_percent << " %\n";
        std::cout << "Peak time tp = " << sm.tp << " s\n";
        std::cout << "Settling time ts(2%) ~ " << sm.ts_2pct << " s\n";
    } else {
        std::cout << "Closed-form step metrics implemented for 0<zeta<1 only.\n";
    }

    std::cout << "\n=== Sweep zeta (fixed wn) ===\n";
    std::cout << "zeta, Mp(%), Mr, wb(rad/s)\n";
    for(double z : {0.15,0.25,0.35,0.50,0.70}){
        auto fm_i = frequency_metrics(z, wn);
        double Mp_i = (z>0.0 && z<1.0)
            ? std::exp(-z*M_PI/std::sqrt(1.0 - z*z))*100.0
            : 0.0;
        std::cout << std::setprecision(2) << z << ", "
                  << std::setprecision(2) << Mp_i << ", "
                  << std::setprecision(4) << fm_i.Mr << ", "
                  << std::setprecision(4) << fm_i.wb << "\n";
    }
    return 0;
}
