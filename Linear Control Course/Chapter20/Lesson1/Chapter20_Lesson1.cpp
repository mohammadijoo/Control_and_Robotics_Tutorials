#include <iostream>

int main() {
    const double zeta = 0.3;
    const double omega_n = 4.0;
    const double k_plant = 1.0;

    const double Kp_1dof = 20.0;

    const double Ky_2dof = 20.0;
    const double Kr_2dof = 20.0;
    const double Tf = 0.2;

    const double dt = 1e-3;
    const double T_end = 2.0;
    const int steps = static_cast<int>(T_end / dt);

    // States: [position, velocity]
    double x1_pos = 0.0, x1_vel = 0.0;
    double x2_pos = 0.0, x2_vel = 0.0;
    double r = 1.0;
    double rf = 0.0;

    for (int k_step = 0; k_step != steps; ++k_step) {
        // Outputs
        double y1 = x1_pos;
        double y2 = x2_pos;

        // 1-DOF control
        double u1 = Kp_1dof * (r - y1);

        // 2-DOF reference filter
        rf += dt * (r - rf) / Tf;
        double u2 = Kr_2dof * rf - Ky_2dof * y2;

        // Plant dynamics for 1-DOF
        double dx1_pos = x1_vel;
        double dx1_vel = -2.0 * zeta * omega_n * x1_vel
                         - omega_n * omega_n * x1_pos
                         + k_plant * u1;
        x1_pos += dt * dx1_pos;
        x1_vel += dt * dx1_vel;

        // Plant dynamics for 2-DOF
        double dx2_pos = x2_vel;
        double dx2_vel = -2.0 * zeta * omega_n * x2_vel
                         - omega_n * omega_n * x2_pos
                         + k_plant * u2;
        x2_pos += dt * dx2_pos;
        x2_vel += dt * dx2_vel;
    }

    std::cout << "Final 1-DOF position: " << x1_pos << std::endl;
    std::cout << "Final 2-DOF position: " << x2_pos << std::endl;

    return 0;
}
