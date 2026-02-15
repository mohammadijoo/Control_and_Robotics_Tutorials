
#include <iostream>
#include <Eigen/Dense>

struct MinimalRobot {
    double x_r; // 1D internal state
    explicit MinimalRobot(double x0=0.0) : x_r(x0) {}

    double sense(double x_e) const {
        return x_e - x_r; // y = h(x_r, x_e)
    }

    double policy(double y, double alpha=0.0, double r=0.0) const {
        double phi = 0.5 * y;
        return alpha * r + (1.0 - alpha) * phi; // u(t)
    }

    void actuate(double u, double dt=0.1) {
        x_r += u * dt; // x_r_dot = u
    }
};

int main() {
    MinimalRobot robot(0.0);
    double x_e = 10.0;

    for(int k=0; k<50; ++k){
        double y = robot.sense(x_e);
        double u = robot.policy(y, 0.2, 0.0);
        robot.actuate(u);
    }
    std::cout << "Final robot position: " << robot.x_r << std::endl;
    return 0;
}
      