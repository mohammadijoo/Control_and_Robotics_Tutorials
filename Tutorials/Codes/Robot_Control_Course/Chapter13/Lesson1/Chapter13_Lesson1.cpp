
#include <iostream>
#include <algorithm>

double nominal_control(double x, double x_ref, double k_p = 2.0)
{
    return -k_p * (x - x_ref);
}

double cbf_filter_1d(double x, double u_nom, double gamma = 5.0)
{
    // CBF constraint: u >= -gamma * x
    double u_min = -gamma * x;
    return std::max(u_nom, u_min);
}

int main()
{
    double dt = 0.001;
    double T = 2.0;
    int N = static_cast<int>(T / dt);

    double x = 0.05;   // initial state
    double x_ref = -1.0;

    for (int k = 0; k < N; ++k)
    {
        double u_nom = nominal_control(x, x_ref);
        double u_safe = cbf_filter_1d(x, u_nom);

        // integrate: x_dot = u_safe
        x += dt * u_safe;

        if (k % 100 == 0)
        {
            std::cout << "t = " << k * dt
                      << ", x = " << x
                      << ", u_nom = " << u_nom
                      << ", u_safe = " << u_safe
                      << std::endl;
        }
    }

    return 0;
}
