
#include <cmath>
#include <iostream>

class DummyLearner {
public:
    double predict(double x) const {
        // Example: cubic nonlinearity as in Python example
        return 0.8 * std::pow(x, 3.0);
    }
};

class ScalarJointWrapper {
public:
    ScalarJointWrapper(double a, double k, double c_scale,
                       const DummyLearner& learner)
        : a_(a), k_(k), c_(c_scale * (a + k)), learner_(learner) {}

    double nominalControl(double x) const {
        return -k_ * x;
    }

    double learnedResidual(double x) const {
        return learner_.predict(x);
    }

    double wrappedControl(double x) const {
        double u_b = nominalControl(x);
        double u_l = learnedResidual(x);
        double bound = c_ * std::abs(x);
        if (std::abs(u_l) > bound) {
            u_l = (u_l > 0.0) ? bound : -bound;
        }
        return u_b + u_l;
    }

private:
    double a_;
    double k_;
    double c_;
    const DummyLearner& learner_;
};

int main() {
    DummyLearner learner;
    ScalarJointWrapper wrapper(1.0, 4.0, 0.5, learner);

    double x = 0.3;
    double u = wrapper.wrappedControl(x);
    std::cout << "x = " << x
              << ", wrapped u = " << u << std::endl;
    return 0;
}
