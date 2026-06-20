    // Chapter19_Lesson2.cpp
    // Separation of Variables & Modal Decomposition (1D Heat Equation, Dirichlet BC)
    //
    // PDE: u_t = alpha * u_xx,  x in (0,L),  u(0,t)=u(L,t)=0
    // Modal series: u(x,t) = sum_{n=1..N} a_n * exp(-alpha*(n*pi/L)^2 * t) * sin(n*pi*x/L)
    //
    // This demo computes a_n numerically (trapezoidal rule) for u0(x)=x(L-x),
    // then evaluates u(x,t) on a grid for a user-specified time.

    #include <iostream>
    #include <vector>
    #include <cmath>
    #include <iomanip>

    static double u0(double x, double L) {
        return x * (L - x);
    }

    static double phi(int n, double x, double L) {
        return std::sin(n * M_PI * x / L);
    }

    static double trapz_coeff(int n, double L, int M) {
        // a_n = (2/L) * ∫_0^L u0(x)*sin(n*pi*x/L) dx
        double h = L / (M - 1);
        double sum = 0.0;
        for (int i = 0; i < M; ++i) {
            double x = i * h;
            double w = (i == 0 || i == M - 1) ? 0.5 : 1.0;
            sum += w * u0(x, L) * phi(n, x, L);
        }
        double integral = h * sum;
        return (2.0 / L) * integral;
    }

    int main() {
        const double L = 1.0;
        const double alpha = 0.02;
        const int N = 40;      // modes
        const int M = 5001;    // integration points
        const int Nx = 200;    // spatial grid points

        double t;
        std::cout << "Enter time t (e.g., 2.5): ";
        std::cin >> t;

        // Compute coefficients
        std::vector<double> a(N+1, 0.0);
        for (int n = 1; n <= N; ++n) {
            a[n] = trapz_coeff(n, L, M);
        }

        // Evaluate u(x,t)
        std::cout << "# x u(x,t)
";
        for (int i = 0; i < Nx; ++i) {
            double x = (L * i) / (Nx - 1);
            double u = 0.0;
            for (int n = 1; n <= N; ++n) {
                double lam = (n * M_PI / L) * (n * M_PI / L);
                u += a[n] * std::exp(-alpha * lam * t) * phi(n, x, L);
            }
            std::cout << std::fixed << std::setprecision(8) << x << " " << u << "
";
        }

        std::cerr << "Computed with N=" << N << " modes.
";
        return 0;
    }
