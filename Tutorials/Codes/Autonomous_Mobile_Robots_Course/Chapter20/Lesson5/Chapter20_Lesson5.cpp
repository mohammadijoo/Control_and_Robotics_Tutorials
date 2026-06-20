\
/* Chapter20_Lesson5.cpp
 * AMR Capstone final demo evaluation + research-style summary (C++17)
 */
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <iomanip>

struct Run {
    double success, time_s, path_m, ref_m, rms_xy, clearance, collisions, latency_ms, energy_wh, map_iou;
};

double mean(const std::vector<double>& x) {
    return std::accumulate(x.begin(), x.end(), 0.0) / x.size();
}
double stdev(const std::vector<double>& x) {
    double m = mean(x), s = 0.0;
    for (double v : x) s += (v - m) * (v - m);
    return std::sqrt(s / (x.size() - 1));
}
double pathEff(const Run& r) { return r.ref_m / r.path_m; }
double energyPerM(const Run& r) { return r.energy_wh / r.path_m; }

struct Agg { double succ, cfree, time, eff, rms, clr, lat, enpm, iou; };

Agg aggregate(const std::vector<Run>& runs) {
    std::vector<double> succ, cfree, time, eff, rms, clr, lat, enpm, iou;
    for (const auto& r : runs) {
        succ.push_back(r.success);
        cfree.push_back(r.collisions == 0 ? 1.0 : 0.0);
        time.push_back(r.time_s); eff.push_back(pathEff(r)); rms.push_back(r.rms_xy);
        clr.push_back(r.clearance); lat.push_back(r.latency_ms); enpm.push_back(energyPerM(r)); iou.push_back(r.map_iou);
    }
    return {mean(succ), mean(cfree), mean(time), mean(eff), mean(rms), mean(clr), mean(lat), mean(enpm), mean(iou)};
}

void pairedCI(const std::vector<double>& p, const std::vector<double>& b, double& m, double& lo, double& hi) {
    std::vector<double> d(p.size());
    for (size_t i = 0; i < p.size(); ++i) d[i] = p[i] - b[i];
    m = mean(d);
    double h = 1.959963984540054 * stdev(d) / std::sqrt((double)d.size());
    lo = m - h; hi = m + h;
}

int main() {
    std::vector<Run> baseline = {
        {1,118.2,28.1,22.0,0.19,0.24,1,31.2,24.8,0.72},
        {1,104.3,24.9,20.0,0.17,0.28,0,28.7,20.1,0.79},
        {0,138.8,30.4,23.0,0.31,0.12,2,39.1,28.5,0.58},
        {1,96.5,22.7,19.0,0.13,0.35,0,24.6,18.4,0.82},
        {1,110.1,26.8,21.0,0.22,0.21,1,33.8,23.7,0.74}
    };
    std::vector<Run> proposed = {
        {1,91.4,24.0,22.0,0.11,0.31,0,22.3,21.2,0.84},
        {1,86.0,21.8,20.0,0.10,0.34,0,20.4,18.8,0.87},
        {1,104.2,25.6,23.0,0.16,0.22,1,24.9,22.1,0.76},
        {1,80.9,20.5,19.0,0.09,0.39,0,18.9,17.3,0.90},
        {1,89.1,23.5,21.0,0.14,0.27,0,21.8,19.6,0.83}
    };

    Agg A = aggregate(baseline), B = aggregate(proposed);
    std::vector<double> tB, tP, iB, iP;
    for (size_t i = 0; i < baseline.size(); ++i) {
        tB.push_back(baseline[i].time_s); tP.push_back(proposed[i].time_s);
        iB.push_back(baseline[i].map_iou); iP.push_back(proposed[i].map_iou);
    }
    double md, lo, hi;
    pairedCI(tP, tB, md, lo, hi);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Baseline success=" << A.succ << ", proposed success=" << B.succ << "\n";
    std::cout << "Baseline time=" << A.time << ", proposed time=" << B.time << "\n";
    std::cout << "Baseline eff=" << A.eff << ", proposed eff=" << B.eff << "\n";
    std::cout << "Baseline IoU=" << A.iou << ", proposed IoU=" << B.iou << "\n";
    std::cout << "Paired delta time (proposed-baseline): " << md << " [" << lo << ", " << hi << "]\n";
    pairedCI(iP, iB, md, lo, hi);
    std::cout << "Paired delta IoU (proposed-baseline): " << md << " [" << lo << ", " << hi << "]\n";
    return 0;
}
