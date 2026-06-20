public class TwoDofPID {
    private double Kp, Ki, Kd;
    private double b, c;
    private double Ts, N;

    private double integ;
    private double prevY, prevR;
    private double dState;

    public TwoDofPID(double Kp, double Ki, double Kd,
                     double b, double c,
                     double Ts, double N) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.b = b;
        this.c = c;
        this.Ts = Ts;
        this.N = N;
        this.integ = 0.0;
        this.prevY = 0.0;
        this.prevR = 0.0;
        this.dState = 0.0;
    }

    public double step(double r, double y) {
        double e = r - y;

        // Proportional term with weighting
        double up = Kp * (b * r - y);

        // Integral term
        integ += Ki * Ts * e;
        double ui = integ;

        // Derivative term with simple filter
        double dr = (r - prevR) / Ts;
        double dy = (y - prevY) / Ts;
        double v = c * dr - dy;

        double alpha = Ts * N;
        dState = (dState + alpha * v) / (1.0 + alpha);
        double ud = Kd * dState;

        prevR = r;
        prevY = y;

        return up + ui + ud;
    }

    public void reset() {
        integ = 0.0;
        dState = 0.0;
        prevR = 0.0;
        prevY = 0.0;
    }
}
