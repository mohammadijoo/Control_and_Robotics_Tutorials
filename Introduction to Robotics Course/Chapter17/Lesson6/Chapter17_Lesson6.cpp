#include <iostream>
#include <cmath>

struct Metrics {
    double lambda_hat;
    double mtbf_hat;
    double availability_hat;
    double mission_success_prob;
};

Metrics deploymentMetrics(double totalUptimeHours,
                          int nFailures,
                          double mttrHours,
                          double missionHours) {
    if (totalUptimeHours <= 0.0) {
        throw std::runtime_error("totalUptimeHours must be positive");
    }
    if (nFailures <= 0) {
        nFailures = 1; // conservative
    }

    double lambdaHat = static_cast<double>(nFailures) / totalUptimeHours;
    double mtbfHat = 1.0 / lambdaHat;
    double availabilityHat = mtbfHat / (mtbfHat + mttrHours);
    double missionSuccess = std::exp(-lambdaHat * missionHours);

    return {lambdaHat, mtbfHat, availabilityHat, missionSuccess};
}

int main() {
    Metrics m = deploymentMetrics(10000.0, 5, 2.0, 8.0);
    std::cout << "lambda_hat: " << m.lambda_hat << "\n";
    std::cout << "MTBF_hat: " << m.mtbf_hat << "\n";
    std::cout << "availability_hat: " << m.availability_hat << "\n";
    std::cout << "mission_success_prob: " << m.mission_success_prob << "\n";
    return 0;
}
      
