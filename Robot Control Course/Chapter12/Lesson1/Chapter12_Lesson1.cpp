
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Vector2d;

struct JointDiscModel {
    Matrix2d Phi;
    Vector2d Gamma;

    JointDiscModel(double J, double b, double Ts) {
        Matrix2d A;
        Vector2d B;
        A << 0.0, 1.0,
               0.0, -b / J;
        B << 0.0,
               1.0 / J;

        // Forward Euler discretization: Phi = I + Ts*A, Gamma = Ts*B
        Matrix2d I = Matrix2d::Identity();
        Phi = I + Ts * A;
        Gamma = Ts * B;
    }

    void step(Vector2d &x, double u) const {
        // x = [q, dq]^T, u = torque
        x = Phi * x + Gamma * u;
    }
};

int main() {
    double J = 0.01;
    double b = 0.1;
    double Ts = 1e-3;

    JointDiscModel model(J, b, Ts);
    Vector2d x;
    x << 0.0, 0.0;  // start at rest
    double u = 1.0;   // constant torque

    for (int k = 0; k < 1000; ++k) { // simulate 1 second
        model.step(x, u);
        // send x(0) as simulated joint angle, etc.
    }
}
