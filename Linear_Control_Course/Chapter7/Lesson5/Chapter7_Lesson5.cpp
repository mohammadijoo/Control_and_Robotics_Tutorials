#include <iostream>
#include <vector>
#include <complex>
#include <Eigen/Dense>

using namespace Eigen;

std::vector<std::pair<std::complex<double>, int> >
rootsWithMultiplicity(const std::vector<double> &den, double tol = 1e-6)
{
    int n = static_cast<int>(den.size()) - 1; // degree
    MatrixXd C = MatrixXd::Zero(n, n);

    // Companion matrix for a_n s^n + ... + a_0
    // First row: -a_{n-1}/a_n, ..., -a_0/a_n
    for (int j = 0; j < n; ++j)
        C(0, j) = -den[j + 1] / den[0]; // den[0] = a_n

    // Subdiagonal ones
    for (int i = 1; i < n; ++i)
        C(i, i - 1) = 1.0;

    EigenSolver<MatrixXd> es(C);
    VectorXcd r = es.eigenvalues();

    std::vector<bool> used(n, false);
    std::vector<std::pair<std::complex<double>, int> > clusters;

    for (int i = 0; i < n; ++i)
    {
        if (used[i]) continue;
        std::complex<double> ri = r(i);
        std::complex<double> sum = ri;
        int multiplicity = 1;
        used[i] = true;

        for (int j = i + 1; j < n; ++j)
        {
            if (!used[j] && std::abs(r(j) - ri) < tol)
            {
                used[j] = true;
                sum += r(j);
                ++multiplicity;
            }
        }
        clusters.emplace_back(sum / static_cast<double>(multiplicity),
                              multiplicity);
    }
    return clusters;
}

std::string classify(const std::vector<double> &den, double tol = 1e-6)
{
    auto roots = rootsWithMultiplicity(den, tol);
    bool unstable = false;
    bool marginal = false;
    for (auto &p : roots)
    {
        std::complex<double> s = p.first;
        int m = p.second;
        double re = s.real();
        if (re > tol)
            unstable = true;
        else if (std::abs(re) <= tol)
        {
            if (m >= 2) unstable = true;
            else marginal = true;
        }
    }
    if (unstable) return "unstable";
    if (marginal) return "marginally stable";
    return "asymptotically stable";
}

int main()
{
    // Example: (s^2 + 1)^2
    std::vector<double> den1 = {1.0, 0.0, 2.0, 0.0, 1.0};
    std::cout << "Example 1: " << classify(den1) << std::endl;

    // In a ROS controller, such a function could be called after
    // updating gains to verify that the closed-loop characteristic
    // polynomial remains acceptable.
    return 0;
}
