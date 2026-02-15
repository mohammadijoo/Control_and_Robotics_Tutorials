public class DiscretePID {
    private final double Kp, Ki, Kd, Ts, alpha, Kaw;
    private final double uMin, uMax;
    private double I, D, ePrev;

    public DiscretePID(double kp, double ki, double kd,
                       double ts, double alpha, double kaw,
                       double uMin, double uMax) {
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
        this.Ts = ts;
        this.alpha = alpha;
        this.Kaw = kaw;
        this.uMin = uMin;
        this.uMax = uMax;
        this.I = 0.0;
        this.D = 0.0;
        this.ePrev = 0.0;
    }

    public double update(double r, double y) {
        double e = r - y;

        // Derivative filter
        D = alpha * D + (1.0 - alpha) * (e - ePrev) / Ts;

        double v = Kp * e + I + Kd * D;

        // Saturation
        double u = Math.max(uMin, Math.min(uMax, v));

        // Anti-windup
        I += Ki * Ts * e + Kaw * (u - v);

        ePrev = e;
        return u;
    }
}

// In a differential-drive robot, one might have two PID instances
// for left and right wheel velocities, using encoders and motor drivers
// provided by the platform's Java robotics library.
