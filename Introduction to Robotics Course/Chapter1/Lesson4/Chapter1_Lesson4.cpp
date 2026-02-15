
#include <iostream>
#include <random>

int main() {
    double a = 1.02, b = 0.05, K = 1.5;
    int N = 2000;

    std::default_random_engine gen(0);
    std::normal_distribution<double> wdist(0.0, 0.02);
    std::normal_distribution<double> vdist(0.0, 0.05);

    double x = 0.0;
    for(int k=0; k<N; ++k){
        double r = 1.0;
        double v = vdist(gen);
        double y = x + v;        // Sense
        double u = -K*y + r;     // Think
        double w = wdist(gen);
        x = a*x + b*u + w;       // Act/Plant
    }
    std::cout << "Final state: " << x << std::endl;
    return 0;
}
      