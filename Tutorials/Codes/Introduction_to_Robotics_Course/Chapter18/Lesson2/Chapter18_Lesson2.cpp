#include <iostream>

struct Observation {
    double d_front;  // meters
};

struct VelocityCommand {
    double v;  // linear velocity (m/s)
    double w;  // angular velocity (rad/s)
};

class ReactiveWallAvoider {
public:
    explicit ReactiveWallAvoider(double d_safe)
        : d_safe_(d_safe) {}

    VelocityCommand computeAction(const Observation& obs) const {
        VelocityCommand cmd;
        if (obs.d_front < d_safe_) {
            cmd.v = 0.0;
            cmd.w = 1.0;
        } else {
            cmd.v = 0.4;
            cmd.w = 0.0;
        }
        return cmd;
    }

private:
    double d_safe_;
};

int main() {
    ReactiveWallAvoider controller(0.6);
    Observation obs{0.4};
    VelocityCommand cmd = controller.computeAction(obs);
    std::cout << "v = " << cmd.v
              << ", w = " << cmd.w << std::endl;
    return 0;
}
      
