#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>

struct Stats {
    int N = 0;
    int successes = 0;
    double sumCost = 0.0;
    double sumCostSq = 0.0;
};

int main() {
    std::ifstream in("planner_logs.txt");
    if (!in) {
        std::cerr << "Could not open log file\n";
        return 1;
    }

    std::unordered_map<std::string, Stats> statsByMethod;
    std::string method;
    int success;
    double cost;

    while (in >> method >> success >> cost) {
        Stats &st = statsByMethod[method];
        st.N += 1;
        st.successes += success;
        st.sumCost += cost;
        st.sumCostSq += cost * cost;
    }

    double alpha = 0.05;
    double z = 1.96; // approx for 95% CI

    for (const auto &kv : statsByMethod) {
        const std::string &m = kv.first;
        const Stats &st = kv.second;
        double p_hat = static_cast<double>(st.successes) / st.N;
        double se_p = std::sqrt(p_hat * (1.0 - p_hat) / st.N);
        double ci_p_low = p_hat - z * se_p;
        double ci_p_high = p_hat + z * se_p;

        double meanCost = st.sumCost / st.N;
        double varCost = (st.sumCostSq - st.sumCost * st.sumCost / st.N)
                         / (st.N - 1);
        double se_c = std::sqrt(varCost / st.N);
        double ci_c_low = meanCost - z * se_c;
        double ci_c_high = meanCost + z * se_c;

        std::cout << "Method " << m << ":\n";
        std::cout << "  N           = " << st.N << "\n";
        std::cout << "  success     = " << p_hat
                  << "  CI = [" << ci_p_low << ", " << ci_p_high << "]\n";
        std::cout << "  mean cost   = " << meanCost
                  << "  CI = [" << ci_c_low << ", " << ci_c_high << "]\n";
    }

    return 0;
}
      
