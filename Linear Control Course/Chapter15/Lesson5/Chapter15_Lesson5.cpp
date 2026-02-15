#include <iostream>
#include <complex>
#include <vector>
#include <cmath>

// Plant: G(s) = 1 / (s (s + 1) (s + 3))
std::complex<double> G_of_s(const std::complex<double> &s)
{
    return 1.0 / (s * (s + 1.0) * (s + 3.0));
}

int main()
{
    double K = 9.0; // test gain (< 12 for stability)
    std::vector<std::complex<double> > nyquistPoints;

    // Logarithmic frequency sweep
    double w_min = 1e-2;
    double w_max = 1e2;
    int N = 400;
    for (int k = 0; k < N; ++k)
    {
        double alpha = static_cast<double>(k) / (N - 1);
        double w = w_min * std::pow(w_max / w_min, alpha);
        std::complex<double> s(0.0, w);      // j * w
        std::complex<double> L = K * G_of_s(s);
        nyquistPoints.push_back(L);
    }

    // Export points for plotting
    for (const auto &z : nyquistPoints)
    {
        std::cout << z.real() << " " << z.imag() << "\n";
    }

    // In a ROS node, one might log the minimal distance of L(jw) to -1
    // as a health metric for the joint controller.
    return 0;
}
