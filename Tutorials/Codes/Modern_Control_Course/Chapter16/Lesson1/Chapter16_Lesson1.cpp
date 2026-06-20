// Chapter16_Lesson1.cpp
// Compile: g++ -std=c++17 Chapter16_Lesson1.cpp -o Chapter16_Lesson1
// Convention: p(s)=s^n+a[n-1]s^(n-1)+...+a[1]s+a[0]

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

Matrix companionA(const Vector& a) {
    const int n = static_cast<int>(a.size());
    Matrix A(n, Vector(n, 0.0));

    for (int i = 0; i < n - 1; ++i) {
        A[i][i + 1] = 1.0;
    }
    for (int j = 0; j < n; ++j) {
        A[n - 1][j] = -a[j];
    }
    return A;
}

Vector inputB(int n) {
    Vector B(n, 0.0);
    B[n - 1] = 1.0;
    return B;
}

Vector matVec(const Matrix& A, const Vector& x) {
    const int n = static_cast<int>(A.size());
    Vector y(n, 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            y[i] += A[i][j] * x[j];
        }
    }
    return y;
}

Matrix controllabilityMatrix(const Matrix& A, const Vector& B) {
    const int n = static_cast<int>(A.size());
    Matrix Q(n, Vector(n, 0.0));
    Vector v = B;

    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            Q[i][k] = v[i];
        }
        v = matVec(A, v);
    }
    return Q;
}

double determinant(Matrix M) {
    const int n = static_cast<int>(M.size());
    double det = 1.0;

    for (int k = 0; k < n; ++k) {
        int pivot = k;
        for (int i = k + 1; i < n; ++i) {
            if (std::fabs(M[i][k]) > std::fabs(M[pivot][k])) {
                pivot = i;
            }
        }

        if (std::fabs(M[pivot][k]) < 1e-12) {
            return 0.0;
        }

        if (pivot != k) {
            std::swap(M[pivot], M[k]);
            det *= -1.0;
        }

        det *= M[k][k];
        const double pivotValue = M[k][k];

        for (int i = k + 1; i < n; ++i) {
            const double factor = M[i][k] / pivotValue;
            for (int j = k; j < n; ++j) {
                M[i][j] -= factor * M[k][j];
            }
        }
    }
    return det;
}

void printMatrix(const std::string& name, const Matrix& M) {
    std::cout << name << " =\n";
    for (const auto& row : M) {
        for (double value : row) {
            std::cout << std::setw(12) << value << " ";
        }
        std::cout << "\n";
    }
}

int main() {
    // Example: p(s)=s^3+4s^2+3s+2
    Vector a{2.0, 3.0, 4.0};  // a0, a1, a2
    Matrix A = companionA(a);
    Vector B = inputB(static_cast<int>(a.size()));
    Matrix Qc = controllabilityMatrix(A, B);

    printMatrix("A", A);
    std::cout << "B = [ ";
    for (double value : B) {
        std::cout << value << " ";
    }
    std::cout << "]^T\n";

    printMatrix("Q_c", Qc);
    std::cout << "det(Q_c) = " << determinant(Qc) << "\n";
    std::cout << "The nonzero determinant confirms controllability in CCF.\n";
    return 0;
}
