public class LowPassFilter {
    private final double alpha;
    private double yPrev = 0.0;

    public LowPassFilter(double Ts, double w_h) {
        this.alpha = Math.exp(-w_h * Ts);
    }

    public double step(double xk) {
        double yk = alpha * yPrev + (1.0 - alpha) * xk;
        yPrev = yk;
        return yk;
    }
}

public class PIDWithRolloff {
    private final double Kp, Ki, Kd;
    private final double Ts;
    private final LowPassFilter measFilter;
    private double integ = 0.0;
    private double measPrev = 0.0;

    public PIDWithRolloff(double Kp, double Ki, double Kd,
                          double Ts, double w_h) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Ts = Ts;
        this.measFilter = new LowPassFilter(Ts, w_h);
    }

    public double step(double ref, double measRaw) {
        double meas = measFilter.step(measRaw);
        double error = ref - meas;
        integ += error * Ts;
        double deriv = (meas - measPrev) / Ts;
        measPrev = meas;

        return Kp * error + Ki * integ - Kd * deriv;
    }
}
