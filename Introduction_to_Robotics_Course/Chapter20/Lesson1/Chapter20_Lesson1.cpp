#include <iostream>

struct PerformanceRequirements {
    double maxSettlingTime_s;
    double maxOvershoot;
    double maxRmsError;
};

struct PerformanceMetrics {
    double settlingTime_s;
    double overshoot;
    double rmsError;
};

struct FeasibilityReport {
    bool settlingTimeOk;
    bool overshootOk;
    bool rmsErrorOk;
};

FeasibilityReport checkFeasibility(const PerformanceRequirements& req,
                                   const PerformanceMetrics& met) {
    FeasibilityReport rep;
    rep.settlingTimeOk = (met.settlingTime_s <= req.maxSettlingTime_s);
    rep.overshootOk    = (met.overshoot      <= req.maxOvershoot);
    rep.rmsErrorOk     = (met.rmsError       <= req.maxRmsError);
    return rep;
}

int main() {
    PerformanceRequirements req {2.0, 0.05, 0.02};
    PerformanceMetrics met {1.9, 0.06, 0.018};

    FeasibilityReport rep = checkFeasibility(req, met);
    std::cout << "settlingTimeOk = " << rep.settlingTimeOk << "\n";
    std::cout << "overshootOk    = " << rep.overshootOk    << "\n";
    std::cout << "rmsErrorOk     = " << rep.rmsErrorOk     << "\n";
    return 0;
}
      
