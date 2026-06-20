#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

// State dimension 1: xdot = -x + u(t)
typedef std::vector<double> state_type;

double u(double t) {
    return (t >= 0.0) ? 1.0 : 0.0;
}

struct dynamics {
    void operator()(const state_type &x, state_type &dxdt, double t) const {
        dxdt[0] = -x[0] + u(t);
    }
};

int main() {
    state_type x(1);
    x[0] = 0.2; // x(0)

    double t0 = 0.0, tf = 5.0, dt = 0.01;

    runge_kutta4<state_type> stepper;
    for (double t = t0; t <= tf; t += dt) {
        stepper.do_step(dynamics(), x, t, dt);
    }

    double y = x[0]; // y = x
    std::cout << "x(tf) = " << x[0] << std::endl;
    std::cout << "y(tf) = " << y << std::endl;
    return 0;
}
