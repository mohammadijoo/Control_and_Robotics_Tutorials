public enum ZNMode {
    P, PI, PID
}

public final class ZNTuning {

    public static double[] znClosedLoop(double Ku, double Tu, ZNMode mode) {
        double Kp, Ti, Td;
        switch (mode) {
            case P:
                Kp = 0.5 * Ku;
                Ti = Double.POSITIVE_INFINITY;
                Td = 0.0;
                break;
            case PI:
                Kp = 0.45 * Ku;
                Ti = Tu / 1.2;
                Td = 0.0;
                break;
            default: // PID
                Kp = 0.60 * Ku;
                Ti = Tu / 2.0;
                Td = Tu / 8.0;
                break;
        }
        double Ki = Double.isInfinite(Ti) ? 0.0 : Kp / Ti;
        double Kd = Kp * Td;
        return new double[]{Kp, Ki, Kd};
    }
}

public class PIDControllerZN {
    private final double Kp;
    private final double Ki;
    private final double Kd;
    private final double dt;
    private final double uMin;
    private final double uMax;

    private double integral = 0.0;
    private double prevError = 0.0;

    public PIDControllerZN(double Kp, double Ki, double Kd,
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
    }

    public double step(double error) {
        integral += error * dt;
        double derivative = (error - prevError) / dt;

        double u = Kp * error + Ki * integral + Kd * derivative;

        if (u > uMax) {
            u = uMax;
            integral -= error * dt;
        } else if (u < uMin) {
            u = uMin;
            integral -= error * dt;
        }

        prevError = error;
        return u;
    }
}

// Example:
// double[] gains = ZNTuning.znClosedLoop(6.0, 1.2, ZNMode.PID);
// PIDControllerZN pid = new PIDControllerZN(gains[0], gains[1], gains[2],
//                                          0.02, -12.0, 12.0);
// double u = pid.step(setpoint - measuredVelocity);
