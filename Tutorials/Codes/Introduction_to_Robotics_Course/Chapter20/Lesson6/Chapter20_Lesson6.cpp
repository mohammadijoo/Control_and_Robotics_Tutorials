#include <cmath>
#include <cstdint>

struct RunMetrics {
    double sum_e2 = 0.0;
    double sum_abs_e_dt = 0.0;
    double sum_u2_dt = 0.0;
    std::uint64_t samples = 0;

    void accumulate(double e, double u, double dt) {
        sum_e2 += e * e;
        sum_abs_e_dt += std::fabs(e) * dt;
        sum_u2_dt += u * u * dt;
        ++samples;
    }

    double rms() const {
        if (samples == 0) return 0.0;
        return std::sqrt(sum_e2 / static_cast<double>(samples));
    }

    double iae() const {
        return sum_abs_e_dt;
    }

    double u2() const {
        return sum_u2_dt;
    }
};

// Example usage in a control loop
void control_loop(RunMetrics& metrics) {
    double t_prev = get_time();
    while (!end_of_run()) {
        double t = get_time();
        double dt = t - t_prev;
        t_prev = t;

        double r = get_reference();
        double y = get_measurement();
        double e = r - y;

        double u = compute_control(e);
        apply_control(u);

        metrics.accumulate(e, u, dt);
    }
}
      
