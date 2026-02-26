
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

int main() {
    double x = 2.0, k = 1.5, alpha = 0.7;
    double T = 5.0, dt = 0.01;
    int n_steps = static_cast<int>(T/dt);

    for(int i=0; i < n_steps; ++i){
        double t = i*dt;
        double uA = -k*x;
        double uH = 0.5*std::sin(2*M_PI*t);
        double u  = alpha*uA + (1-alpha)*uH;

        x += dt*u; // x_dot = u
    }
    std::cout << "Final state: " << x << std::endl;
    return 0;
}
      