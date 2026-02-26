public class TwoDofPIDController {
    private double Kp, Ki, Kd;
    private double beta, gamma;
    private double Ts;

    private double eInt = 0.0;
    private double rPrev = 0.0, rPrev2 = 0.0;
    private double yPrev = 0.0, yPrev2 = 0.0;

    public TwoDofPIDController(double Kp, double Ki, double Kd,
                               double beta, double gamma, double Ts) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.beta = beta;
        this.gamma = gamma;
        this.Ts = Ts;
    }

    public double update(double r, double y) {
        double e = r - y;
        eInt += e * Ts;

        double rBeta = beta * r;
        double rGamma = gamma * r;

        double dr = (rGamma - 2.0 * rPrev + rPrev2) / (Ts * Ts);
        double dy = (y - 2.0 * yPrev + yPrev2) / (Ts * Ts);

        double u = Kp * (rBeta - y) + Ki * eInt + Kd * (dr - dy);

        rPrev2 = rPrev;
        rPrev = rGamma;
        yPrev2 = yPrev;
        yPrev = y;

        return u;
    }
}
