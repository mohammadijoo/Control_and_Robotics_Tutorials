/*
Chapter17_Lesson1.cpp

Observable Canonical Form (OCF) construction for a SISO transfer function.

Compile:
    g++ -std=c++17 Chapter17_Lesson1.cpp -o Chapter17_Lesson1

This version uses only the C++ standard library. For larger systems, prefer
Eigen or Armadillo for numerical linear algebra.
*/

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

struct StateSpace {
    Matrix A;
    Vector B;
    Vector C;
    double D;
};

void print_matrix(const std::string& name, const Matrix& M) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double v : row) {
            std::cout << std::setw(12) << v << " ";
        }
        std::cout << "\n";
    }
}

void print_vector(const std::string& name, const Vector& v) {
    std::cout << name << " = [ ";
    for (double x : v) {
        std::cout << x << " ";
    }
    std::cout << "]\n";
}

Matrix multiply(const Matrix& A, const Matrix& B) {
    int n = static_cast<int>(A.size());
    int m = static_cast<int>(B[0].size());
    int p = static_cast<int>(B.size());
    Matrix C(n, Vector(m, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < p; ++k) {
            for (int j = 0; j < m; ++j) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

Matrix identity(int n) {
    Matrix I(n, Vector(n, 0.0));
    for (int i = 0; i < n; ++i) {
        I[i][i] = 1.0;
    }
    return I;
}

int rank_gaussian(Matrix M, double tol = 1e-10) {
    int rows = static_cast<int>(M.size());
    int cols = static_cast<int>(M[0].size());
    int r = 0;
    for (int c = 0; c < cols && r < rows; ++c) {
        int pivot = r;
        for (int i = r + 1; i < rows; ++i) {
            if (std::fabs(M[i][c]) > std::fabs(M[pivot][c])) {
                pivot = i;
            }
        }
        if (std::fabs(M[pivot][c]) < tol) {
            continue;
        }
        std::swap(M[pivot], M[r]);
        double div = M[r][c];
        for (int j = c; j < cols; ++j) {
            M[r][j] /= div;
        }
        for (int i = 0; i < rows; ++i) {
            if (i == r) {
                continue;
            }
            double factor = M[i][c];
            for (int j = c; j < cols; ++j) {
                M[i][j] -= factor * M[r][j];
            }
        }
        ++r;
    }
    return r;
}

StateSpace observable_canonical_form(Vector den, Vector num) {
    if (den.empty() || std::fabs(den[0]) < 1e-14) {
        throw std::runtime_error("Denominator leading coefficient must be nonzero.");
    }

    double leading = den[0];
    for (double& v : den) {
        v /= leading;
    }
    for (double& v : num) {
        v /= leading;
    }

    int n = static_cast<int>(den.size()) - 1;
    if (n <= 0) {
        throw std::runtime_error("Denominator degree must be positive.");
    }
    if (static_cast<int>(num.size()) > n + 1) {
        throw std::runtime_error("Expected a proper transfer function.");
    }

    Vector num_pad(n + 1, 0.0);
    int offset = n + 1 - static_cast<int>(num.size());
    for (int i = 0; i < static_cast<int>(num.size()); ++i) {
        num_pad[offset + i] = num[i];
    }

    double D = num_pad[0];
    Vector remainder(n + 1, 0.0);
    for (int i = 0; i <= n; ++i) {
        remainder[i] = num_pad[i] - D * den[i];
    }

    Matrix A(n, Vector(n, 0.0));
    for (int i = 1; i < n; ++i) {
        A[i][i - 1] = 1.0;
    }
    for (int i = 0; i < n; ++i) {
        A[i][n - 1] = -den[n - i];
    }

    Vector B(n, 0.0);
    for (int i = 0; i < n; ++i) {
        B[i] = remainder[n - i];
    }

    Vector C(n, 0.0);
    C[n - 1] = 1.0;

    return {A, B, C, D};
}

Matrix observability_matrix(const Matrix& A, const Vector& C) {
    int n = static_cast<int>(A.size());
    Matrix O(n, Vector(n, 0.0));
    Matrix Ak = identity(n);

    for (int row = 0; row < n; ++row) {
        for (int j = 0; j < n; ++j) {
            double value = 0.0;
            for (int k = 0; k < n; ++k) {
                value += C[k] * Ak[k][j];
            }
            O[row][j] = value;
        }
        Ak = multiply(Ak, A);
    }
    return O;
}

int main() {
    // G(s) = (2 s^2 + 5 s + 3) / (s^3 + 4 s^2 + 6 s + 8)
    Vector den = {1.0, 4.0, 6.0, 8.0};
    Vector num = {2.0, 5.0, 3.0};

    StateSpace sys = observable_canonical_form(den, num);

    print_matrix("A_o", sys.A);
    print_vector("B_o", sys.B);
    print_vector("C_o", sys.C);
    std::cout << "D_o = " << sys.D << "\n";

    Matrix O = observability_matrix(sys.A, sys.C);
    print_matrix("O_o", O);
    std::cout << "rank(O_o) = " << rank_gaussian(O) << "\n";

    return 0;
}
