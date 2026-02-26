#include <stdint.h>
#include <stdio.h>

// Hardware abstraction (stubs)
double read_velocity();               // rad/s
void set_motor_command(double u);     // normalized command [-1, 1]
void delay_ms(uint32_t ms);           // blocking delay

struct Metrics {
    double e_inf;
    double Ts;
};

Metrics run_step_test(double Kp, double v_ref, double tol, double T_max) {
    const double dt = 0.01; // 10 ms control period
    uint32_t steps = static_cast<uint32_t>(T_max / dt);
    double v = 0.0;
    double v_inf = v_ref;
    double t = 0.0;

    uint32_t last_out_of_band = 0;

    for (uint32_t k = 0; k < steps; ++k) {
        v = read_velocity();
        double e = v_ref - v;
        double u = Kp * e;
        if (u > 1.0) u = 1.0;
        if (u < -1.0) u = -1.0;
        set_motor_command(u);

        double err_band = (v_inf != 0.0) ?
                          (v - v_inf) / v_inf : 0.0;
        if (fabs(err_band) > tol) {
            last_out_of_band = k;
        }

        delay_ms(static_cast<uint32_t>(dt * 1000.0));
        t += dt;
    }

    Metrics m;
    m.e_inf = v_inf - v;
    m.Ts = last_out_of_band * 0.01;
    return m;
}

int main() {
    double Kp_list[] = {0.5, 1.0, 2.0};
    for (double Kp : Kp_list) {
        Metrics m = run_step_test(Kp, 5.0, 0.02, 5.0);
        printf("Kp=%.2f, e_inf=%.3f, Ts=%.2fs\n",
               Kp, m.e_inf, m.Ts);
    }
    return 0;
}
      
