// Chapter16_Lesson5.cpp
// Discrete-time stability and time response demo (C++17, no external libraries)
#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <iomanip>

struct ComplexPair {
    double real;
    double imag;
};

double det2(const std::array<std::array<double,2>,2>& A) {
    return A[0][0]*A[1][1] - A[0][1]*A[1][0];
}

double trace2(const std::array<std::array<double,2>,2>& A) {
    return A[0][0] + A[1][1];
}

ComplexPair eig_pair_2x2(const std::array<std::array<double,2>,2>& A, double& lambda1_abs, double& lambda2_abs) {
    double tr = trace2(A);
    double det = det2(A);
    double disc = tr*tr - 4.0*det;

    if (disc >= 0.0) {
        double s = std::sqrt(disc);
        double l1 = 0.5*(tr + s);
        double l2 = 0.5*(tr - s);
        lambda1_abs = std::fabs(l1);
        lambda2_abs = std::fabs(l2);
        return {0.0, 0.0};
    } else {
        double re = 0.5*tr;
        double im = 0.5*std::sqrt(-disc);
        double mag = std::sqrt(re*re + im*im);
        lambda1_abs = mag;
        lambda2_abs = mag;
        return {re, im};
    }
}

// Jury test for a2 z^2 + a1 z + a0 (with a2 > 0)
bool jury_second_order(double a2, double a1, double a0) {
    if (a2 <= 0.0) return false;
    // Necessary and sufficient for second-order Schur stability:
    // p(1) > 0, p(-1) > 0 for normalized orientation, and |a0| < a2
    // for polynomial a2 z^2 + a1 z + a0
    double p1 = a2 + a1 + a0;
    double pm1 = a2 - a1 + a0;
    return (std::fabs(a0) < a2) && (p1 > 0.0) && (pm1 > 0.0);
}

int main() {
    std::array<std::array<double,2>,2> A{{ {1.5770, -0.6724},
                                           {1.0000,  0.0000} }};
    std::array<double,2> B{{1.0, 0.0}};
    std::array<double,2> C{{0.0676, 0.0604}};
    double D = 0.0;

    // Characteristic polynomial of A: z^2 - tr(A) z + det(A)
    double a2 = 1.0;
    double a1 = -trace2(A);
    double a0 = det2(A);

    double mag1=0.0, mag2=0.0;
    ComplexPair cp = eig_pair_2x2(A, mag1, mag2);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Characteristic polynomial: z^2 + (" << a1 << ") z + (" << a0 << ")\n";
    std::cout << "Jury stable (2nd order test)? " << (jury_second_order(a2, a1, a0) ? "true" : "false") << "\n";
    std::cout << "Eigenvalue magnitudes: " << mag1 << ", " << mag2 << "\n";
    if (cp.imag != 0.0) {
        std::cout << "Complex conjugate poles: " << cp.real << " +/- j" << cp.imag << "\n";
    }

    // Step response simulation
    std::array<double,2> x{{0.0, 0.0}};
    const int N = 60;
    std::vector<double> y;
    y.reserve(N);

    for (int k=0; k<N; ++k) {
        double u = 1.0;
        double yk = C[0]*x[0] + C[1]*x[1] + D*u;
        y.push_back(yk);

        std::array<double,2> xn{};
        xn[0] = A[0][0]*x[0] + A[0][1]*x[1] + B[0]*u;
        xn[1] = A[1][0]*x[0] + A[1][1]*x[1] + B[1]*u;
        x = xn;
    }

    std::cout << "\nFirst 15 step-response samples:\n";
    for (int k=0; k<15; ++k) {
        std::cout << "k=" << std::setw(2) << k << "  y=" << y[k] << "\n";
    }

    // Approximate settling sample for 2% band around final value 1.0
    int k_settle = -1;
    for (int k=0; k<N; ++k) {
        bool inside = true;
        for (int j=k; j<N; ++j) {
            if (std::fabs(y[j] - 1.0) > 0.02) { inside = false; break; }
        }
        if (inside) { k_settle = k; break; }
    }
    std::cout << "\n2% settling sample index = " << k_settle << "\n";
    return 0;
}
