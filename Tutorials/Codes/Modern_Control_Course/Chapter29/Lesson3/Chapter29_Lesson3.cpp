/*
Chapter29_Lesson3.cpp

Numerical experiments for Lesson 3:
Stability notions for continuous-time linear time-varying systems.

Build:
    g++ -std=c++17 -O2 Chapter29_Lesson3.cpp -o Chapter29_Lesson3

Run:
    ./Chapter29_Lesson3
*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

struct Matrix2 {
    double v[2][2];
};

Matrix2 zero_matrix() {
    Matrix2 M;
    M.v[0][0] = 0.0; M.v[0][1] = 0.0;
    M.v[1][0] = 0.0; M.v[1][1] = 0.0;
    return M;
}

Matrix2 identity_matrix() {
    Matrix2 M = zero_matrix();
    M.v[0][0] = 1.0;
    M.v[1][1] = 1.0;
    return M;
}

Matrix2 add(Matrix2 A, Matrix2 B) {
    Matrix2 C = zero_matrix();
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            C.v[i][j] = A.v[i][j] + B.v[i][j];
        }
    }
    return C;
}

Matrix2 scale(Matrix2 A, double s) {
    Matrix2 C = zero_matrix();
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            C.v[i][j] = s * A.v[i][j];
        }
    }
    return C;
}

Matrix2 multiply(Matrix2 A, Matrix2 B) {
    Matrix2 C = zero_matrix();
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                C.v[i][j] += A.v[i][k] * B.v[k][j];
            }
        }
    }
    return C;
}

double frobenius_norm(Matrix2 A) {
    double s = 0.0;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            s += A.v[i][j] * A.v[i][j];
        }
    }
    return std::sqrt(s);
}

Matrix2 A_of_t(double t) {
    Matrix2 A = zero_matrix();
    double k = 1.0 + 0.20 * std::sin(t);
    double c = 0.80 + 0.10 * std::cos(2.0 * t);
    A.v[0][0] = 0.0;
    A.v[0][1] = 1.0;
    A.v[1][0] = -k;
    A.v[1][1] = -c;
    return A;
}

Matrix2 f(double t, Matrix2 Phi) {
    return multiply(A_of_t(t), Phi);
}

Matrix2 rk4_phi(double t0, double t1, double h) {
    if (t1 < t0) {
        throw std::invalid_argument("This routine assumes t1 >= t0.");
    }
    int n = static_cast<int>(std::ceil((t1 - t0) / h));
    if (n == 0) {
        return identity_matrix();
    }

    double h_eff = (t1 - t0) / static_cast<double>(n);
    double t = t0;
    Matrix2 Phi = identity_matrix();

    for (int step = 0; step < n; ++step) {
        Matrix2 K1 = f(t, Phi);
        Matrix2 K2 = f(t + 0.5 * h_eff, add(Phi, scale(K1, 0.5 * h_eff)));
        Matrix2 K3 = f(t + 0.5 * h_eff, add(Phi, scale(K2, 0.5 * h_eff)));
        Matrix2 K4 = f(t + h_eff, add(Phi, scale(K3, h_eff)));

        Matrix2 incr = add(add(K1, scale(K2, 2.0)), add(scale(K3, 2.0), K4));
        Phi = add(Phi, scale(incr, h_eff / 6.0));
        t += h_eff;
    }
    return Phi;
}

double phi_uniform_exponential_scalar(double t, double t0) {
    return std::exp(-0.5 * (t - t0) + 0.25 * (std::cos(t0) - std::cos(t)));
}

double phi_uniform_stable_not_uniform_attractive(double t, double t0) {
    return (1.0 + t0) / (1.0 + t);
}

int main() {
    std::cout << std::fixed << std::setprecision(6);

    std::cout << "Scalar uniformly exponentially stable example\n";
    double M = std::exp(0.5);
    double alpha = 0.5;
    for (double tau : std::vector<double>{0.0, 1.0, 2.0, 5.0, 10.0}) {
        double max_ratio = 0.0;
        for (int i = 0; i <= 40; ++i) {
            double t0 = 0.5 * i;
            double phi = std::abs(phi_uniform_exponential_scalar(t0 + tau, t0));
            double bound = M * std::exp(-alpha * tau);
            max_ratio = std::max(max_ratio, phi / bound);
        }
        std::cout << "tau=" << tau << ", max |Phi|/bound = " << max_ratio << "\n";
    }

    std::cout << "\nUniformly stable but not uniformly attractive example\n";
    for (double T : std::vector<double>{1.0, 5.0, 20.0}) {
        std::cout << "T=" << T << ": ";
        for (double t0 : std::vector<double>{0.0, 10.0, 100.0, 1000.0, 10000.0}) {
            std::cout << phi_uniform_stable_not_uniform_attractive(t0 + T, t0) << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\n2x2 transition-matrix Frobenius norm estimates\n";
    for (double tau : std::vector<double>{1.0, 2.0, 5.0, 10.0}) {
        double max_norm = 0.0;
        for (int i = 0; i <= 5; ++i) {
            double t0 = static_cast<double>(i);
            Matrix2 Phi = rk4_phi(t0, t0 + tau, 0.005);
            max_norm = std::max(max_norm, frobenius_norm(Phi));
        }
        std::cout << "tau=" << tau << ", max sampled ||Phi||_F = " << max_norm << "\n";
    }

    return 0;
}
