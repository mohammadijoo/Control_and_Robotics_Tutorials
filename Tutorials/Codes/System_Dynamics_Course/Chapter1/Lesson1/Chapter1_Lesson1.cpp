
#include <iostream>
#include <boost/numeric/odeint.hpp>

using state_type = double;

// dx/dt = a * x + b * u, with u(t) = 1
void rhs(const state_type &x, state_type &dxdt, const double t)
{
    const double a = -1.0;
    const double b = 1.0;
    const double u = 1.0;
    dxdt = a * x + b * u;
}

int main()
{
    state_type x = 0.0;                 // initial condition
    double t0 = 0.0;
    double tf = 5.0;
    double dt = 0.01;

    // integrate using a simple stepper
    boost::numeric::odeint::integrate(rhs, x, t0, tf, dt);

    std::cout << "x(" << tf << ") = " << x << std::endl;
    return 0;
}
      