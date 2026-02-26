#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>

struct Sample {
    double e;
};

int main() {
    std::ifstream in("robot_logs.csv");
    if (!in) {
        std::cerr << "Cannot open file\n";
        return 1;
    }

    std::string header;
    std::getline(in, header); // skip header

    // Map trial_id to list of error samples
    std::unordered_map<int, std::vector<Sample>> data;

    std::string line;
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::string token;
        double time, y_ref, y_meas, u;
        int trial_id;

        std::getline(ss, token, ','); time = std::stod(token);
        std::getline(ss, token, ','); y_ref = std::stod(token);
        std::getline(ss, token, ','); y_meas = std::stod(token);
        std::getline(ss, token, ','); u = std::stod(token);
        std::getline(ss, token, ','); trial_id = std::stoi(token);

        Sample s;
        s.e = y_ref - y_meas;
        data[trial_id].push_back(s);
    }

    std::vector<double> J_values;
    for (auto &kv : data) {
        const auto &samples = kv.second;
        double sum_sq = 0.0;
        for (const auto &s : samples) {
            sum_sq += s.e * s.e;
        }
        double J_rms = std::sqrt(sum_sq / samples.size());
        J_values.push_back(J_rms);
        std::cout << "Trial " << kv.first << " RMS error = " << J_rms << "\n";
    }

    int N = static_cast<int>(J_values.size());
    double mean = 0.0;
    for (double v : J_values) mean += v;
    mean /= N;

    double var = 0.0;
    for (double v : J_values) {
        double d = v - mean;
        var += d * d;
    }
    var /= (N - 1);
    double stddev = std::sqrt(var);

    std::cout << "Mean RMS error = " << mean << "\n";
    std::cout << "Std of RMS error = " << stddev << "\n";

    return 0;
}
      
