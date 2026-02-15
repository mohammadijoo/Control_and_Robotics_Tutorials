#include <iostream>
#include <vector>
#include <cppad/cppad.hpp>

using CppAD::AD;

template <typename Scalar>
std::vector<Scalar> f(const std::vector<Scalar>& x,
                       Scalar tau, Scalar I,
                       Scalar b, Scalar k)
{
    Scalar q  = x[0];
    Scalar dq = x[1];
    Scalar ddq = (tau - b * dq - k * q) / I;
    std::vector<Scalar> dx(2);
    dx[0] = dq;
    dx[1] = ddq;
    return dx;
}

template <typename Scalar>
std::vector<Scalar> stepEuler(const std::vector<Scalar>& x,
                               Scalar tau, Scalar I,
                               Scalar b, Scalar k, Scalar dt)
{
    std::vector<Scalar> dx = f(x, tau, I, b, k);
    std::vector<Scalar> xnext(2);
    xnext[0] = x[0] + dt * dx[0];
    xnext[1] = x[1] + dt * dx[1];
    return xnext;
}

int main()
{
    using ADScalar = AD<double>;

    // Independent variable: I (inertia parameter)
    std::vector<ADScalar> I_vec(1);
    I_vec[0] = 0.5;
    CppAD::Independent(I_vec);

    ADScalar I = I_vec[0];
    ADScalar b = 0.1;
    ADScalar k = 1.0;
    ADScalar dt = 0.01;
    ADScalar tau = 1.0;

    std::vector<ADScalar> x(2);
    x[0] = 0.0; // q
    x[1] = 0.0; // dq

    // Simulate N steps and accumulate loss
    const int N = 100;
    ADScalar L = 0.0;
    for (int kstep = 0; kstep <= N; ++kstep) {
        ADScalar q_ref = 0.5 * CppAD::sin(2.0 * M_PI * dt * kstep);
        ADScalar err = x[0] - q_ref;
        L += 0.5 * err * err;
        if (kstep < N) {
            x = stepEuler(x, tau, I, b, k, dt);
        }
    }

    // Declare dependent variable
    std::vector<ADScalar> L_vec(1);
    L_vec[0] = L;
    CppAD::ADFun<double> tape(I_vec, L_vec);

    // Compute dL/dI at I = 0.5
    std::vector<double> I_val(1), grad(1);
    I_val[0] = 0.5;
    grad = tape.Jacobian(I_val); // since output dimension is 1
    std::cout << "Loss gradient dL/dI = " << grad[0] << std::endl;

    return 0;
}
      
