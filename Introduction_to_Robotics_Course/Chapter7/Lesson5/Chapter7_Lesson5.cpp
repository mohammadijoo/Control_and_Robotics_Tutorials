#include <iostream>
#include <vector>
#include <random>
#include <cmath>

int main() {
    double fs = 100.0, Ts = 1.0/fs;
    int N = 200;
    std::vector<double> x(N), b(N), y(N), yq(N), t(N);

    // Random generators
    std::mt19937 gen(0);
    std::normal_distribution<double> noise(0.0, 0.05);
    std::normal_distribution<double> drift(0.0, 1e-4);

    // Quantizer parameters
    int B = 10;
    double ymin = -2.0, ymax = 2.0;
    double Delta = (ymax - ymin) / std::pow(2.0, B);

    // Simulate
    for(int k=0; k<N; ++k){
        t[k] = k*Ts;
        x[k] = std::sin(2*M_PI*3*t[k]);               // assume band-limited here
        if(k==0) b[k] = 0.0; else b[k] = b[k-1] + drift(gen);
        y[k] = x[k] + b[k] + noise(gen);

        // Quantize
        yq[k] = Delta * std::round(y[k]/Delta);
    }

    // Estimate bias from first 20 samples
    double b_hat = 0.0;
    for(int k=0; k<20; ++k) b_hat += yq[k];
    b_hat /= 20.0;

    std::cout << "Delta=" << Delta << ", b_hat=" << b_hat << std::endl;
    return 0;
}
