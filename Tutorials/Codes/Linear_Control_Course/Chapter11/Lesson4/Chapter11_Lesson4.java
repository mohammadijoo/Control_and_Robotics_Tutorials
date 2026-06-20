public class PIDController {
    private double Kp, Ki, Kd;
    private double dt;
    private double integral = 0.0;
    private double prevError = 0.0;
    private boolean firstCall = true;
    private double uMin, uMax;

    public PIDController(double Kp, double Ki, double Kd,
                         double dt, double uMin, double uMax) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.dt = dt;
        this.uMin = uMin;
        this.uMax = uMax;
    }

    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        firstCall = true;
    }

    public double update(double r, double y) {
        double e = r - y;

        // Integral
        integral += e * dt;

        // Derivative
        double d;
        if (firstCall) {
            d = 0.0;
            firstCall = false;
        } else {
            d = (e - prevError) / dt;
        }

        double u = Kp * e + Ki * integral + Kd * d;

        // Saturation
        if (u < uMin) u = uMin;
        if (u > uMax) u = uMax;

        prevError = e;
        return u;
    }
}

/*
 * In Java-based robotics (e.g., WPILib for FRC robots),
 * a similar PIDController class is used to control wheel speeds,
 * arm positions, etc., again relying on the parallel (Kp, Ki, Kd) structure.
 */
