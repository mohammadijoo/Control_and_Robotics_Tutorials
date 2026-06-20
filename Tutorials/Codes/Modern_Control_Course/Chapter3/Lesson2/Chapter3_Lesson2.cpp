#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using Vec = Eigen::VectorXd;
using Mat = Eigen::MatrixXd;

Mat A_of_t(double t) {
    Mat A(2,2);
    A << 0.0, 1.0,
        -2.0 - 0.2*std::sin(t), -0.4;
    return A;
}

Vec b_of_t(double t) {
    Vec b(2);
    b << 0.0, 0.5*std::cos(t);
    return b;
}

Vec f(double t, const Vec& x) {
    return A_of_t(t)*x + b_of_t(t);
}

Vec rk4_step(double t, const Vec& x, double h) {
    Vec k1 = f(t, x);
    Vec k2 = f(t + 0.5*h, x + 0.5*h*k1);
    Vec k3 = f(t + 0.5*h, x + 0.5*h*k2);
    Vec k4 = f(t + h,     x + h*k3);
    return x + (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}

int main() {
    double t0 = 0.0, tf = 10.0, h = 0.01;
    int N = static_cast<int>((tf - t0)/h);

    Vec x(2);
    x << 1.0, 0.0;

    double t = t0;
    for (int k = 0; k < N; ++k) {
        x = rk4_step(t, x, h);
        t += h;
    }

    std::cout << "Final x = [" << x(0) << ", " << x(1) << "]\n";
    return 0;
}
      
