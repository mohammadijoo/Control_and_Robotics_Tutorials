#include <iostream>
#include <vector>
#include <cmath>

using std::size_t;

void centralized_step(std::vector<double> &p,
                      const std::vector<double> &p0,
                      double k_gain, double dt)
{
    const size_t N = p.size();
    double p_avg0 = 0.0;
    for (size_t i = 0; i < N; ++i) {
        p_avg0 += p0[i];
    }
    p_avg0 /= static_cast<double>(N);

    for (size_t i = 0; i < N; ++i) {
        double u_i = -k_gain * (p[i] - p_avg0);
        p[i] += dt * u_i;
    }
}

void decentralized_step(std::vector<double> &p,
                        const std::vector<std::vector<size_t> > &neighbors,
                        double k_gain, double dt)
{
    const size_t N = p.size();
    std::vector<double> u(N, 0.0);

    for (size_t i = 0; i < N; ++i) {
        double s = 0.0;
        for (size_t j : neighbors[i]) {
            s += (p[i] - p[j]);
        }
        u[i] = -k_gain * s;
    }

    for (size_t i = 0; i < N; ++i) {
        p[i] += dt * u[i];
    }
}

int main()
{
    const size_t N = 5;
    double dt = 0.05;
    double k_gain = 1.0;
    std::vector<double> p0{ -2.0, -1.0, 0.5, 1.5, 3.0 };
    std::vector<double> p_central = p0;
    std::vector<double> p_decent  = p0;

    // ring neighbors
    std::vector<std::vector<size_t> > neighbors(N);
    for (size_t i = 0; i < N; ++i) {
        neighbors[i].push_back((i + N - 1) % N);
        neighbors[i].push_back((i + 1) % N);
    }

    const int steps = 200;
    for (int k = 0; k < steps; ++k) {
        centralized_step(p_central, p0, k_gain, dt);
        decentralized_step(p_decent, neighbors, k_gain, dt);
    }

    std::cout << "Centralized final:" << std::endl;
    for (double v : p_central) std::cout << v << " ";
    std::cout << std::endl;

    std::cout << "Decentralized final:" << std::endl;
    for (double v : p_decent) std::cout << v << " ";
    std::cout << std::endl;

    return 0;
}
      
