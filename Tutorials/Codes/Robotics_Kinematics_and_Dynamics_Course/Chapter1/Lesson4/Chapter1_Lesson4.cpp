#include <iostream>
#include <array>
#include <boost/numeric/odeint.hpp>

using state_type = std::array<double, 2>;

// Parameters
const double J = 0.01;
const double b = 0.1;
const double k = 1.0;

// ODE: x[0] = q, x[1] = qd
void joint_ode(const state_type &x, state_type &dxdt, double t)
{
    (void)t; // unused
    double q  = x[0];
    double qd = x[1];
    double u  = 1.0; // constant torque
    double qdd = (u - b * qd - k * q) / J;
    dxdt[0] = qd;
    dxdt[1] = qdd;
}

int main()
{
    using namespace boost::numeric::odeint;

    state_type x = {0.0, 0.0}; // initial state

    auto observer = [](const state_type &x, double t) {
        std::cout << t << " " << x[0] << " " << x[1] << std::endl;
    };

    size_t steps = integrate(joint_ode, x, 0.0, 5.0, 0.01, observer);
    std::cerr << "Total steps: " << steps << std::endl;

    return 0;
}
      
