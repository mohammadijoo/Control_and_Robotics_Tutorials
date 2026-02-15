
public class JointPIDController {
    private double Kp;
    private double Kd;
    private double Ki;

    private double integral = 0.0;
    private double ePrev = 0.0;
    private double dFiltered = 0.0;
    private double alpha = 0.2; // derivative filter weight

    public JointPIDController(double Kp, double Kd, double Ki) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
    }

    public double update(double q, double dq,
                         double qd,
                         double dt) {
        double e = qd - q;
        double de = (e - ePrev) / dt;
        dFiltered = (1.0 - alpha) * dFiltered + alpha * de;

        integral += e * dt;

        // naive anti-windup
        double zMax = 10.0;
        if (integral > zMax) integral = zMax;
        if (integral < -zMax) integral = -zMax;

        double tau = Kp * e + Kd * dFiltered + Ki * integral;
        ePrev = e;
        return tau;
    }
}
