#include <vector>
#include <cmath>
#include <iostream>

struct MetricAccumulator {
    int nTrials = 0;
    int nSuccess = 0;
    double sumCost = 0.0;
    double sumSqCost = 0.0;

    void addTrial(bool success, double costIfSuccess) {
        ++nTrials;
        if (success) {
            ++nSuccess;
            sumCost += costIfSuccess;
            sumSqCost += costIfSuccess * costIfSuccess;
        }
    }

    double successRate() const {
        if (nTrials == 0) return 0.0;
        return static_cast<double>(nSuccess) / static_cast<double>(nTrials);
    }

    int nSuccessSamples() const { return nSuccess; }

    double meanCost() const {
        if (nSuccess == 0) return std::numeric_limits<double>::quiet_NaN();
        return sumCost / static_cast<double>(nSuccess);
    }

    double varCost() const {
        if (nSuccess <= 1) return std::numeric_limits<double>::quiet_NaN();
        double mu = meanCost();
        return (sumSqCost - static_cast<double>(nSuccess) * mu * mu)
               / static_cast<double>(nSuccess - 1);
    }

    // Approximate 95% CI for mean cost using normal approximation
    std::pair<double,double> meanCostCI95() const {
        if (nSuccess <= 1) {
            return {std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()};
        }
        double mu = meanCost();
        double var = varCost();
        double se = std::sqrt(var / static_cast<double>(nSuccess));
        double z = 1.96; // approx for 95%
        return {mu - z * se, mu + z * se};
    }
};

int main() {
    MetricAccumulator acc;
    // Example loop over benchmark runs (e.g. OMPL's PlannerTerminationCondition)
    for (int i = 0; i < 100; ++i) {
        bool success = /* run planner and check if goal reached */;
        double pathLength = /* compute path length if success */;
        acc.addTrial(success, pathLength);
    }

    std::cout << "Success rate: " << acc.successRate() << std::endl;
    std::cout << "Mean cost (success only): " << acc.meanCost() << std::endl;
    auto ci = acc.meanCostCI95();
    std::cout << "95% CI for mean cost: [" << ci.first
              << ", " << ci.second << "]" << std::endl;
    return 0;
}
      
