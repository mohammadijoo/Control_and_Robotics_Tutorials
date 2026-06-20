
#include <iostream>
#include <vector>
#include <numeric>

struct Joint {
    std::string name;
    int fi;
};

int mobility_spatial(int L, const std::vector<Joint>& joints) {
    int J = (int)joints.size();
    int sum_f = 0;
    for (const auto& j : joints) sum_f += j.fi;
    return 6 * (L - 1 - J) + sum_f;
}

int main() {
    std::vector<Joint> serial;
    for (int i = 0; i < 6; ++i) serial.push_back({"R"+std::to_string(i+1), 1});
    std::cout << "Serial M = " << mobility_spatial(7, serial) << std::endl;

    std::vector<Joint> fivebar(5, {"R", 1});
    std::cout << "Five-bar M (spatial formula) = "
              << mobility_spatial(5, fivebar) << std::endl;
    return 0;
}
      