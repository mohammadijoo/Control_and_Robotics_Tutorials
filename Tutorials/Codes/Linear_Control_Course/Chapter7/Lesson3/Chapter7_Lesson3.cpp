#include <vector>
#include <iostream>
#include <cmath>

struct RouthResult {
    std::vector<std::vector<double>> table;
    int n_rhp;
};

RouthResult routhArray(const std::vector<double>& coeffs, double eps = 1e-9)
{
    int n = static_cast<int>(coeffs.size()) - 1; // degree
    int m = (n + 2) / 2; // ceil((n+1)/2)
    std::vector<std::vector<double>> R(n + 1, std::vector<double>(m, 0.0));

    // Fill first two rows
    int idx = 0;
    for (int j = 0; j < m && idx < static_cast<int>(coeffs.size()); ++j, idx += 2) {
        R[0][j] = coeffs[idx]; // a_n, a_(n-2), ...
    }
    idx = 1;
    for (int j = 0; j < m && idx < static_cast<int>(coeffs.size()); ++j, idx += 2) {
        R[1][j] = coeffs[idx]; // a_(n-1), a_(n-3), ...
    }

    // Build subsequent rows
    for (int i = 2; i <= n; ++i) {
        if (std::fabs(R[i - 1][0]) < eps) {
            R[i - 1][0] = eps;
        }
        for (int j = 0; j < m - 1; ++j) {
            double a = R[i - 1][0];
            double b = R[i - 2][0];
            double c = R[i - 2][j + 1];
            double d = R[i - 1][j + 1];
            R[i][j] = (a * c - b * d) / a;
        }
    }

    // Count sign changes in first column
    int sign_changes = 0;
    auto sign = [eps](double x) {
        if (x > eps) return 1;
        if (x < -eps) return -1;
        return 1; // treat near-zero as small positive
    };
    for (int i = 0; i < n; ++i) {
        int s1 = sign(R[i][0]);
        int s2 = sign(R[i + 1][0]);
        if (s1 * s2 < 0) {
            ++sign_changes;
        }
    }

    return {R, sign_changes};
}

int main()
{
    // Example: p(s) = s^3 + 6 s^2 + 8 s + K with K = 10
    std::vector<double> coeffs = {1.0, 6.0, 8.0, 10.0};
    RouthResult res = routhArray(coeffs);

    std::cout << "Routh array:\n";
    for (const auto& row : res.table) {
        for (double val : row) {
            std::cout << val << "\t";
        }
        std::cout << "\n";
    }
    std::cout << "Number of RHP roots: " << res.n_rhp << "\n";

    if (res.n_rhp == 0) {
        std::cout << "Closed-loop is asymptotically stable.\n";
    } else {
        std::cout << "Closed-loop is unstable.\n";
    }

    return 0;
}
