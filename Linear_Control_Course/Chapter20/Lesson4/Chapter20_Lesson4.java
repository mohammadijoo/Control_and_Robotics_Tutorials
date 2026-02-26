public class TwoDofPID {
    private double kp, ki, kd;
    private double b, c;
    private double dt;

    private double integral;
    private double yPrev;
    private double rPrev;

    public TwoDofPID(double kp, double ki, double kd,
                     double b, double c, double dt) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.b = b;
        this.c = c;
        this.dt = dt;
        reset();
    }

    public double update(double r, double y) {
        double e = r - y;
        integral += e * dt;

        double v = c * r - y;
        double vPrev = c * rPrev - yPrev;
        double dv = (v - vPrev) / dt;

        double up = kp * (b * r - y);
        double ui = ki * integral;
        double ud = kd * dv;

        yPrev = y;
        rPrev = r;

        return up + ui + ud;
    }

    public void reset() {
        integral = 0.0;
        yPrev = 0.0;
        rPrev = 0.0;
    }
}
