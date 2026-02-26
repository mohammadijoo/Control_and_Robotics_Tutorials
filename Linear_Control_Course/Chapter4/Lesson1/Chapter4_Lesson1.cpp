#include <iostream>
#include <vector>
#include <complex>

class TransferFunction {
public:
    // G(s) = num(s) / den(s)
    std::vector<double> num;
    std::vector<double> den;

    TransferFunction(const std::vector<double>& n,
                     const std::vector<double>& d)
        : num(n), den(d) {}

    std::complex<double> eval(const std::complex<double>& s) const {
        std::complex<double> N(0.0, 0.0), D(0.0, 0.0);
        for (std::size_t i = 0; i < num.size(); ++i) {
            int p = static_cast<int>(num.size() - 1 - i);
            N += num[i] * std::pow(s, p);
        }
        for (std::size_t i = 0; i < den.size(); ++i) {
            int p = static_cast<int>(den.size() - 1 - i);
            D += den[i] * std::pow(s, p);
        }
        return N / D;
    }
};

int main() {
    double K = 2.0;
    double T = 0.5;

    std::vector<double> num{K};
    std::vector<double> den{T, 1.0};

    TransferFunction G(num, den);

    // Evaluate G(j*omega) at omega = 1 rad/s (for analysis)
    std::complex<double> s(0.0, 1.0);
    std::complex<double> Gjw = G.eval(s);

    std::cout << "G(j*1) = " << Gjw << std::endl;

    // In a robotics context, this class can be used inside a ROS node
    // to approximate continuous dynamics or to design simple controllers.
    return 0;
}
