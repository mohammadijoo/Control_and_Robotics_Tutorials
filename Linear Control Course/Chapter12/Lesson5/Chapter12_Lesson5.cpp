class DiscretePID {
public:
    DiscretePID(double Kp_, double Ki_, double Kd_,
                double Ts_, double alpha_, double Kaw_,
                double u_min_, double u_max_)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_), Ts(Ts_),
          alpha(alpha_), Kaw(Kaw_),
          u_min(u_min_), u_max(u_max_),
          I(0.0), D(0.0), e_prev(0.0)
    {}

    double update(double r, double y) {
        double e = r - y;

        // Derivative filter
        D = alpha * D + (1.0 - alpha) * (e - e_prev) / Ts;

        // Unsaturated output
        double v = Kp * e + I + Kd * D;

        // Saturation
        double u = v;
        if (u > u_max) u = u_max;
        if (u < u_min) u = u_min;

        // Anti-windup
        I += Ki * Ts * e + Kaw * (u - v);

        e_prev = e;
        return u;
    }

private:
    double Kp, Ki, Kd, Ts, alpha, Kaw;
    double u_min, u_max;
    double I, D, e_prev;
};

// Example usage in a loop with Ts sampling time:
// DiscretePID pid(2.0, 50.0, 0.001, 0.001, 0.9, 20.0, -12.0, 12.0);
// while (running) {
//     double theta = readEncoderJoint();
//     double r = getReferencePosition();
//     double u = pid.update(r, theta);
//     writeMotorVoltage(u);
// }
