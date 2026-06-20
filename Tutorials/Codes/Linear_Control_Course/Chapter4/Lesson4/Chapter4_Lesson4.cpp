#include <vector>
#include <iostream>
#include <algorithm>

struct TransferFunction {
    std::vector<double> num; // numerator coefficients (descending powers of s)
    std::vector<double> den; // denominator coefficients
};

std::vector<double> convolve(const std::vector<double>& a,
                               const std::vector<double>& b) {
    std::vector<double> c(a.size() + b.size() - 1, 0.0);
    for (std::size_t i = 0; i < a.size(); ++i) {
        for (std::size_t j = 0; j < b.size(); ++j) {
            c[i + j] += a[i] * b[j];
        }
    }
    return c;
}

std::vector<double> addPoly(const std::vector<double>& a,
                              const std::vector<double>& b) {
    std::size_t n = std::max(a.size(), b.size());
    std::vector<double> c(n, 0.0);
    // Right-align coefficients
    for (std::size_t i = 0; i < a.size(); ++i) {
        c[n - a.size() + i] += a[i];
    }
    for (std::size_t i = 0; i < b.size(); ++i) {
        c[n - b.size() + i] += b[i];
    }
    return c;
}

TransferFunction series(const TransferFunction& G1,
                       const TransferFunction& G2) {
    TransferFunction G;
    G.num = convolve(G1.num, G2.num);
    G.den = convolve(G1.den, G2.den);
    return G;
}

TransferFunction parallel(const TransferFunction& G1,
                         const TransferFunction& G2) {
    TransferFunction G;
    std::vector<double> den = convolve(G1.den, G2.den);
    std::vector<double> num1 = convolve(G1.num, G2.den);
    std::vector<double> num2 = convolve(G2.num, G1.den);
    G.den = den;
    G.num = addPoly(num1, num2);
    return G;
}

// Negative feedback: G / (1 + G H)
TransferFunction feedback(const TransferFunction& G,
                          const TransferFunction& H) {
    TransferFunction T;
    std::vector<double> num = convolve(G.num, H.den);
    std::vector<double> den1 = convolve(G.den, H.den);
    std::vector<double> den2 = convolve(G.num, H.num);
    T.num = num;
    T.den = addPoly(den1, den2);
    return T;
}

int main() {
    // Example: DC motor-like plant Gp(s) = K / (J s^2 + b s)
    double J = 0.01;
    double b = 0.1;
    double K = 1.0;
    double Kc = 20.0; // proportional controller

    TransferFunction Gp{ {K}, {J, b, 0.0} }; // num, den
    TransferFunction Gc{ {Kc}, {1.0} };

    // Forward path
    TransferFunction Gforward = series(Gc, Gp);

    // Sensor H(s) = 1
    TransferFunction H{ {1.0}, {1.0} };

    // Closed-loop
    TransferFunction Tcl = feedback(Gforward, H);

    std::cout << "Closed-loop numerator: ";
    for (double c : Tcl.num) std::cout << c << " ";
    std::cout << "\nClosed-loop denominator: ";
    for (double c : Tcl.den) std::cout << c << " ";
    std::cout << std::endl;

    return 0;
}
