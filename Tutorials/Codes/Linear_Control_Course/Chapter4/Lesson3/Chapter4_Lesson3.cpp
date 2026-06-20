#include <vector>
#include <stdexcept>

struct TransferFunction {
    // Y(s)/U(s) = num(s)/den(s)
    std::vector<double> num;
    std::vector<double> den;

    // Polynomial addition (for same length, padding if needed)
    static std::vector<double> polyAdd(const std::vector<double>& a,
                                         const std::vector<double>& b) {
        std::size_t n = std::max(a.size(), b.size());
        std::vector<double> c(n, 0.0);
        for (std::size_t i = 0; i < n; ++i) {
            double av = (i < a.size()) ? a[a.size()-1-i] : 0.0;
            double bv = (i < b.size()) ? b[b.size()-1-i] : 0.0;
            c[n-1-i] = av + bv;
        }
        return c;
    }

    // Polynomial multiplication
    static std::vector<double> polyMul(const std::vector<double>& a,
                                         const std::vector<double>& b) {
        std::vector<double> c(a.size() + b.size() - 1, 0.0);
        for (std::size_t i = 0; i < a.size(); ++i)
            for (std::size_t j = 0; j < b.size(); ++j)
                c[i + j] += a[i] * b[j];
        return c;
    }

    // Closed-loop unity feedback: T(s) = G(s)/(1 + G(s))
    TransferFunction unityFeedback() const {
        // 1 is represented as den(s)/den(s)
        std::vector<double> numG = num;
        std::vector<double> denG = den;
        std::vector<double> numOne = denG;
        std::vector<double> denOne = denG;

        // 1 + G(s) = (denG + numG) / denG
        std::vector<double> numSum = polyAdd(numOne, numG);
        std::vector<double> denSum = denG;  // common denominator

        // T = G / (1 + G) = (numG/denG) / (numSum/denSum)
        //   = (numG * denSum) / (denG * numSum)
        std::vector<double> numT = polyMul(numG, denSum);
        std::vector<double> denT = polyMul(denG, numSum);
        return {numT, denT};
    }
};

// In a robotics application, G(s) could be the joint servo transfer function,
// and unityFeedback() would give the closed-loop behavior for analysis.
