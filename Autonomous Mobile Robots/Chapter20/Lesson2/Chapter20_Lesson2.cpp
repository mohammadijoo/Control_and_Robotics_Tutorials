// Chapter20_Lesson2.cpp
// Sensor Suite Selection (given hardware) - C++/Eigen implementation
// Compile (Linux): g++ -std=c++17 Chapter20_Lesson2.cpp -O2 -I /usr/include/eigen3 -o sensor_select

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>

struct SensorSpec {
    std::string name;
    double rateHz;
    double powerW;
    double bandwidthKbps;
    double cpuMs;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
    bool mandatory;
};

Eigen::MatrixXd infoUpdate(const Eigen::MatrixXd& P, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) {
    Eigen::MatrixXd Iprior = P.inverse();
    Eigen::MatrixXd Imeas  = H.transpose() * R.inverse() * H;
    return (Iprior + Imeas).inverse();
}

struct Metrics {
    double traceP;
    double logdetInfo;
    double powerW;
    double bandwidthKbps;
    double cpuLoad;
    double meanLatencyMs;
};

Metrics evaluateSuite(const std::vector<SensorSpec>& suite, const Eigen::MatrixXd& P0, double dt = 0.1) {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(5, 5);
    Q.diagonal() << 0.03 * dt, 0.03 * dt, 0.01 * dt, 0.05 * dt, 0.002 * dt;
    Eigen::MatrixXd P = P0 + Q;

    double totalPower = 0.0, totalBw = 0.0, cpuLoad = 0.0, latencyMs = 0.0;
    for (const auto& s : suite) {
        P = infoUpdate(P, s.H, s.R);
        totalPower += s.powerW;
        totalBw += s.bandwidthKbps;
        cpuLoad += (s.cpuMs * s.rateHz) / 1000.0;
        latencyMs += 1000.0 / s.rateHz + s.cpuMs;
    }

    double sign = 0.0, logdetP = 0.0;
    Eigen::LLT<Eigen::MatrixXd> llt(P);
    if (llt.info() == Eigen::Success) {
        const auto L = llt.matrixL();
        for (int i = 0; i < L.rows(); ++i) { logdetP += 2.0 * std::log(L(i, i)); }
    } else {
        logdetP = std::log(P.determinant());
    }

    Metrics m;
    m.traceP = P.trace();
    m.logdetInfo = -logdetP;   // log(det(P^{-1}))
    m.powerW = totalPower;
    m.bandwidthKbps = totalBw;
    m.cpuLoad = cpuLoad;
    m.meanLatencyMs = latencyMs / std::max<size_t>(1, suite.size());
    return m;
}

double score(const Metrics& m) {
    return 1.0 * m.logdetInfo
         - 0.35 * m.traceP
         - 0.03 * m.powerW
         - 0.0008 * m.bandwidthKbps
         - 1.8 * m.cpuLoad
         - 0.01 * m.meanLatencyMs;
}

SensorSpec makeSensor(
    const std::string& name, double rateHz, double powerW, double bw, double cpuMs,
    const Eigen::MatrixXd& H, const Eigen::VectorXd& sigma, bool mandatory = false
) {
    SensorSpec s;
    s.name = name; s.rateHz = rateHz; s.powerW = powerW; s.bandwidthKbps = bw; s.cpuMs = cpuMs; s.H = H;
    s.R = sigma.array().square().matrix().asDiagonal();
    s.mandatory = mandatory;
    return s;
}

int main() {
    using Eigen::MatrixXd;
    using Eigen::VectorXd;
    std::vector<SensorSpec> sensors;

    MatrixXd Henc(2, 5); Henc << 0,0,0,1,0,  0,0,1,0,1;
    VectorXd senc(2); senc << 0.03, 0.01;
    sensors.push_back(makeSensor("wheel_encoder", 50.0, 0.5, 20.0, 0.3, Henc, senc, true));

    MatrixXd Himu(2, 5); Himu << 0,0,1,0,1,  0,0,0,1,0;
    VectorXd simu(2); simu << 0.015, 0.01;
    sensors.push_back(makeSensor("imu", 200.0, 0.8, 120.0, 0.4, Himu, simu, true));

    MatrixXd Hlidar(3, 5); Hlidar << 1,0,0,0,0,  0,1,0,0,0,  0,0,1,0,0;
    VectorXd slidar(3); slidar << 0.05, 0.05, 0.02;
    sensors.push_back(makeSensor("2d_lidar", 10.0, 8.0, 1500.0, 12.0, Hlidar, slidar));

    MatrixXd Hcam(3, 5); Hcam << 1,0,0,0,0,  0,1,0,0,0,  0,0,1,0,0;
    VectorXd scam(3); scam << 0.08, 0.08, 0.03;
    sensors.push_back(makeSensor("mono_camera", 30.0, 2.5, 2500.0, 18.0, Hcam, scam));

    MatrixXd Hdepth(4, 5); Hdepth << 1,0,0,0,0,  0,1,0,0,0,  0,0,1,0,0,  0,0,0,1,0;
    VectorXd sdepth(4); sdepth << 0.06, 0.06, 0.03, 0.08;
    sensors.push_back(makeSensor("depth_camera", 15.0, 4.5, 5000.0, 20.0, Hdepth, sdepth));

    MatrixXd Hgnss(2, 5); Hgnss << 1,0,0,0,0,  0,1,0,0,0;
    VectorXd sgnss(2); sgnss << 0.03, 0.03;
    sensors.push_back(makeSensor("gnss_rtk", 5.0, 2.0, 40.0, 1.5, Hgnss, sgnss));

    std::vector<int> mandatoryIdx, optionalIdx;
    for (int i = 0; i < static_cast<int>(sensors.size()); ++i) {
        if (sensors[i].mandatory) mandatoryIdx.push_back(i);
        else optionalIdx.push_back(i);
    }

    MatrixXd P0 = MatrixXd::Zero(5, 5);
    P0.diagonal() << 1.5, 1.5, 0.5, 0.8, 0.2;

    const double PWR_BUDGET = 14.0;
    const double BW_BUDGET = 7000.0;
    const double CPU_BUDGET = 0.80;
    const double LAT_BUDGET = 120.0;

    double bestScore = -1e18;
    std::vector<std::string> bestNames;
    Metrics bestMetrics{};

    const int M = static_cast<int>(optionalIdx.size());
    for (int mask = 0; mask < (1 << M); ++mask) {
        std::vector<SensorSpec> suite;
        for (int idx : mandatoryIdx) suite.push_back(sensors[idx]);
        for (int j = 0; j < M; ++j) {
            if (mask & (1 << j)) suite.push_back(sensors[optionalIdx[j]]);
        }

        Metrics m = evaluateSuite(suite, P0);
        bool feasible = (m.powerW <= PWR_BUDGET) &&
                        (m.bandwidthKbps <= BW_BUDGET) &&
                        (m.cpuLoad <= CPU_BUDGET) &&
                        (m.meanLatencyMs <= LAT_BUDGET);
        if (!feasible) continue;

        double J = score(m);
        if (J > bestScore) {
            bestScore = J;
            bestMetrics = m;
            bestNames.clear();
            for (const auto& s : suite) bestNames.push_back(s.name);
        }
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Best feasible suite\\n";
    for (const auto& n : bestNames) std::cout << "  - " << n << "\\n";
    std::cout << "score = " << bestScore << "\\n";
    std::cout << "trace(P+) = " << bestMetrics.traceP << "\\n";
    std::cout << "logdetInfo = " << bestMetrics.logdetInfo << "\\n";
    std::cout << "powerW = " << bestMetrics.powerW << "\\n";
    std::cout << "bandwidthKbps = " << bestMetrics.bandwidthKbps << "\\n";
    std::cout << "cpuLoad = " << bestMetrics.cpuLoad << "\\n";
    std::cout << "meanLatencyMs = " << bestMetrics.meanLatencyMs << "\\n";
    return 0;
}
