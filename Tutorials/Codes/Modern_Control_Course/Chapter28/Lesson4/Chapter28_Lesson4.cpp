// Chapter28_Lesson4.cpp
// Scalar closed-form LQR illustration for qualitative Q/R effects.
// Compile example:
//   g++ -std=c++17 Chapter28_Lesson4.cpp -O2 -o Chapter28_Lesson4

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

struct CaseData {
    std::string name;
    double q;
    double r;
};

int main() {
    // Scalar unstable or lightly stable plant: x_dot = a x + b u, u = -k x.
    // For q > 0, r > 0, the stabilizing scalar LQR solution gives:
    // k = (a + sqrt(a^2 + b^2 q/r)) / b, assuming b > 0.
    const double a = 0.4;
    const double b = 1.0;
    const double x0 = 1.0;

    std::vector<CaseData> cases = {
        {"balanced", 1.0, 1.0},
        {"larger_state_weight", 25.0, 1.0},
        {"larger_input_weight", 1.0, 25.0},
        {"small_input_weight", 1.0, 0.04}
    };

    std::cout << std::fixed << std::setprecision(5);
    std::cout << "case, q, r, k, closed_loop_lambda, settling_indicator, initial_u\n";

    for (const auto& c : cases) {
        double k = (a + std::sqrt(a * a + b * b * c.q / c.r)) / b;
        double lambda_cl = a - b * k;
        double settling_indicator = -1.0 / lambda_cl;  // smaller means faster decay
        double u0 = -k * x0;

        std::cout << c.name << ", "
                  << c.q << ", "
                  << c.r << ", "
                  << k << ", "
                  << lambda_cl << ", "
                  << settling_indicator << ", "
                  << u0 << "\n";
    }

    return 0;
}
