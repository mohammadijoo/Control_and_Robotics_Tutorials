class AntiWindupPI {
public:
    AntiWindupPI(double Kp, double Ki, double T_aw,
                 double u_min, double u_max, double Ts)
        : Kp_(Kp), Ki_(Ki), T_aw_(T_aw),
          u_min_(u_min), u_max_(u_max), Ts_(Ts),
          xI_(0.0), u_(0.0), v_(0.0) {}

    double step(double r, double y) {
        double e = r - y;

        // Unsaturated controller output
        v_ = Kp_ * e + Ki_ * xI_;

        // Saturation
        u_ = v_;
        if (u_ > u_max_) u_ = u_max_;
        if (u_ < u_min_) u_ = u_min_;

        // Back-calculation anti-windup
        double e_aw = u_ - v_;
        xI_ += Ts_ * (e + (1.0 / T_aw_) * e_aw);

        return u_;
    }

    void reset(double xI0 = 0.0) { xI_ = xI0; }

private:
    double Kp_, Ki_, T_aw_;
    double u_min_, u_max_, Ts_;
    double xI_;
    double u_, v_;
};

// Example usage in a robot joint control loop (pseudo-code):
// AntiWindupPI pi(8.0, 20.0, 0.05, -5.0, 5.0, 0.001);
// while (running) {
//     double y = readJointPosition();
//     double r = desiredJointPosition();
//     double u = pi.step(r, y);
//     sendActuatorCommand(u);
// }
