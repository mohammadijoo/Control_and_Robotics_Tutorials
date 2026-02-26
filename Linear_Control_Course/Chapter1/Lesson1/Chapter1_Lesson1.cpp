#include <iostream>
#include <vector>

int main() {
    double a = 1.0;
    double b = 2.0;
    double K = 3.0;
    double r0 = 1.0;

    double h = 0.001;
    double t_final = 2.0;
    int N = static_cast<int>(t_final / h);

    std::vector<double> y(N + 1, 0.0);
    std::vector<double> u(N + 1, 0.0);

    y[0] = 0.0;

    for (int k = 0; k < N; ++k) {
        double e_k = r0 - y[k];
        u[k] = K * e_k;
        double dy = -(a + b * K) * y[k] + b * K * r0;
        y[k + 1] = y[k] + h * dy;
    }
    u[N] = u[N - 1];

    std::cout << "Final output y(T) = " << y[N] << std::endl;

    return 0;
}

/*
In a robotic joint controller using ROS control, similar logic would be
implemented inside an update() method, using Eigen for vector/matrix
computations and hardware interfaces for reading joint states and
writing commands.
*/
