#include <iostream>
#include <vector>

int main() {
    const double a = 1.0;
    const double b = 1.0;
    const double K = 2.0;
    const double h = 0.01;
    const int    N = 1000;
    const double r = 1.0;

    std::vector<double> y_open(N + 1, 0.0);
    std::vector<double> y_closed(N + 1, 0.0);

    const double u_open = a / b * r;

    for (int k = 0; k < N; ++k) {
        // Open-loop update
        y_open[k + 1] = y_open[k] +
                        h * (-a * y_open[k] + b * u_open);

        // Closed-loop update
        double e_k = r - y_closed[k];
        double u_closed = K * e_k;
        y_closed[k + 1] = y_closed[k] +
                          h * (-a * y_closed[k] + b * u_closed);
    }

    std::cout << "k,time,y_open,y_closed\n";
    for (int k = 0; k <= N; ++k) {
        double t = k * h;
        std::cout << k << ","
                  << t << ","
                  << y_open[k] << ","
                  << y_closed[k] << "\n";
    }

    return 0;
}
