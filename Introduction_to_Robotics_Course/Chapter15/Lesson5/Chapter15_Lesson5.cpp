#include <vector>
#include <stdexcept>
#include <iostream>

struct LogEntry {
    int group;     // 0 or 1
    double cost;   // nonnegative cost for this interaction
};

double groupRisk(const std::vector<LogEntry> &log, int g) {
    double sum = 0.0;
    std::size_t count = 0;
    for (const auto &e : log) {
        if (e.group == g) {
            sum += e.cost;
            ++count;
        }
    }
    if (count == 0) {
        throw std::runtime_error("No samples for requested group");
    }
    return sum / static_cast<double>(count);
}

int main() {
    std::vector<LogEntry> log = {
        {0, 0.0}, {0, 1.0}, {1, 0.2}, {1, 0.1}, {1, 0.7}, {0, 0.3}
    };
    double r0 = groupRisk(log, 0);
    double r1 = groupRisk(log, 1);
    double disparity = std::abs(r1 - r0);

    std::cout << "r0 = " << r0 << ", r1 = " << r1
              << ", disparity = " << disparity << std::endl;
    const double eps = 0.05;
    if (disparity > eps) {
        std::cout << "Warning: fairness constraint violated." << std::endl;
    }
    return 0;
}
      
