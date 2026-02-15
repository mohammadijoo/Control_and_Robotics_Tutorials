#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <tuple>

struct StateStepHash {
    std::size_t operator()(const std::pair<double,int>& s) const {
        // Simple hash: combine rounded x and step
        long xi = std::lround(s.first * 100); // 2 decimals
        return std::hash<long>{}(xi) ^ (std::hash<int>{}(s.second) << 1);
    }
};

int main() {
    const double T = 0.1;
    const double L = 1.0;
    const double u_max = 0.5;
    const int N_horizon = 10;
    const double dx = 0.1;

    std::vector<double> controls = {-u_max, 0.0, u_max};
    std::vector<double> initial_states = {0.0};

    std::queue<std::pair<double,int>> q;
    std::unordered_set<std::pair<double,int>, StateStepHash> visited;

    for (double x0 : initial_states) {
        q.push({x0, 0});
        visited.insert({x0, 0});
    }

    auto succ = [&](double x) {
        std::vector<std::pair<double,bool>> result;
        for (double u : controls) {
            double x_next = x + T * u;
            if (x_next <= L && x_next >= -L) {
                double x_disc = dx * std::round(x_next / dx);
                result.push_back({x_disc, true});
            } else {
                result.push_back({0.0, false}); // unsafe
            }
        }
        return result;
    };

    bool safe = true;
    while (!q.empty() && safe) {
        auto [x, step] = q.front();
        q.pop();
        if (step == N_horizon) continue;

        for (auto& pair : succ(x)) {
            double x_next = pair.first;
            bool inside = pair.second;
            if (!inside) {
                std::cout << "Unsafe successor from x=" << x << std::endl;
                safe = false;
                break;
            }
            std::pair<double,int> node{x_next, step + 1};
            if (visited.find(node) == visited.end()) {
                visited.insert(node);
                q.push(node);
            }
        }
    }

    std::cout << "Abstraction declared SAFE up to horizon "
              << N_horizon << ": " << std::boolalpha << safe << std::endl;

    return 0;
}
      
