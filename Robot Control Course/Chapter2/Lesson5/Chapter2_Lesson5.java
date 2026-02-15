
public class JointServo {
    private double Kp;
    private double Kd;
    private double Ki;
    private double tauMin;
    private double tauMax;
    private double integral;

    public JointServo(double kp, double kd, double ki,
                      double tauMin, double tauMax) {
        this.Kp = kp;
        this.Kd = kd;
        this.Ki = ki;
        this.tauMin = tauMin;
        this.tauMax = tauMax;
        this.integral = 0.0;
    }

    private double gravityTorque(double q) {
        double m = 1.0;
        double ell = 0.5;
        double g = 9.81;
        return m * g * ell * Math.sin(q);
    }

    public double update(double q, double dq,
                         double qd, double dqd,
                         double dt) {
        double e = qd - q;
        double de = dqd - dq;

        integral += e * dt;
        double Imax = 5.0;
        if (integral > Imax) integral = Imax;
        if (integral < -Imax) integral = -Imax;

        double tau = Kp * e + Kd * de + Ki * integral
                     + gravityTorque(q);

        if (tau > tauMax) tau = tauMax;
        if (tau < tauMin) tau = tauMin;
        return tau;
    }
}
