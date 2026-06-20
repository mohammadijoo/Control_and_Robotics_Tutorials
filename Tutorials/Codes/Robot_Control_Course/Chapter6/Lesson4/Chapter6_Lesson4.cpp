
#include <iostream>

class Admittance1D {
public:
    Admittance1D(double M_a, double D_a, double K_a, double x0, double dt)
        : M_a_(M_a), D_a_(D_a), K_a_(K_a), x0_(x0), dt_(dt),
          z1_(0.0), z2_(0.0)
    {
        const double alpha = dt_ / M_a_;
        A11_ = 1.0 - dt_ * alpha * K_a_;
        A12_ = dt_ * (1.0 - alpha * D_a_);
        A21_ = -alpha * K_a_;
        A22_ = 1.0 - alpha * D_a_;
        B1_ = dt_ * alpha;
        B2_ = alpha;
    }

    void step(double F_ext, double& x_r, double& xdot_r) {
        const double z1_next = A11_ * z1_ + A12_ * z2_ + B1_ * F_ext;
        const double z2_next = A21_ * z1_ + A22_ * z2_ + B2_ * F_ext;
        z1_ = z1_next;
        z2_ = z2_next;
        x_r = z1_ + x0_;
        xdot_r = z2_;
    }

private:
    double M_a_, D_a_, K_a_, x0_, dt_;
    double z1_, z2_;
    double A11_, A12_, A21_, A22_, B1_, B2_;
};

int main() {
    Admittance1D ctrl(3.0, 20.0, 50.0, 0.0, 0.001);
    double x_r = 0.0, xdot_r = 0.0;
    const double F_ext = 10.0;

    for (int k = 0; k < 5000; ++k) {
        ctrl.step(F_ext, x_r, xdot_r);
        if (k % 1000 == 0) {
            std::cout << "k=" << k
                      << ", x_r=" << x_r
                      << ", xdot_r=" << xdot_r
                      << std::endl;
        }
    }
    return 0;
}
