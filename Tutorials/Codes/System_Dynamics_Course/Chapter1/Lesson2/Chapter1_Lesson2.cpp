
#include <vector>
#include <cmath>

// Abstract base for SISO discrete-time systems
class DiscreteSystem {
public:
    virtual ~DiscreteSystem() {}
    // takes full input trajectory u and returns output y
    virtual std::vector<double> apply(const std::vector<double>& u) const = 0;
};

// Static linear system: y[k] = k_gain * u[k]
class StaticLinear : public DiscreteSystem {
public:
    explicit StaticLinear(double k_gain) : k_gain_(k_gain) {}
    std::vector<double> apply(const std::vector<double>& u) const override {
        std::vector<double> y(u.size());
        for (std::size_t k = 0; k < u.size(); ++k) {
            y[k] = k_gain_ * u[k];
        }
        return y;
    }
private:
    double k_gain_;
};

// Dynamic linear system: y[k+1] = a*y[k] + b*u[k], y[0] = 0
class FirstOrderDynamic : public DiscreteSystem {
public:
    FirstOrderDynamic(double a, double b) : a_(a), b_(b) {}
    std::vector<double> apply(const std::vector<double>& u) const override {
        std::size_t N = u.size();
        std::vector<double> y(N, 0.0);
        for (std::size_t k = 0; k + 1 < N; ++k) {
            y[k + 1] = a_ * y[k] + b_ * u[k];
        }
        return y;
    }
private:
    double a_, b_;
};
      