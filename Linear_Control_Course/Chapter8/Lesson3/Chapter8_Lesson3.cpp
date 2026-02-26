#include <iostream>
#include <limits>
#include <cmath>

enum class InputType { Step, Ramp, Parabolic };

struct ErrorConstants {
    double Kp;  // position error constant
    double Kv;  // velocity error constant
    double Ka;  // acceleration error constant
};

double steadyStateError(InputType input, const ErrorConstants& k)
{
    switch (input) {
    case InputType::Step:
        if (std::isinf(k.Kp)) {
            return 0.0;
        }
        return 1.0 / (1.0 + k.Kp);

    case InputType::Ramp:
        if (k.Kv <= 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        return 1.0 / k.Kv;

    case InputType::Parabolic:
        if (k.Ka <= 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        return 1.0 / k.Ka;

    default:
        return std::numeric_limits<double>::quiet_NaN();
    }
}

int main()
{
    // Example: type-1 joint servo with Kp finite, Kv finite, Ka = 0
    ErrorConstants jointServo{20.0, 5.0, 0.0};

    std::cout << "Step e_ss      = "
              << steadyStateError(InputType::Step, jointServo) << "\n";
    std::cout << "Ramp e_ss      = "
              << steadyStateError(InputType::Ramp, jointServo) << "\n";
    std::cout << "Parabolic e_ss = "
              << steadyStateError(InputType::Parabolic, jointServo) << "\n";

    return 0;
}
