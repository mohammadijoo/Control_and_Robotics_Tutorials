public class ClampedPIController {
    private double Kp;
    private double Ki;
    private double Ts;
    private double uMin;
    private double uMax;
    private double xI;

    public ClampedPIController(double Kp, double Ki, double Ts,
                               double uMin, double uMax) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Ts = Ts;
        this.uMin = uMin;
        this.uMax = uMax;
        this.xI = 0.0;
    }

    public double step(double r, double y) {
        double e = r - y;

        // Tentative integral update
        double xI_candidate = xI + Ts * e;

        // PI output before saturation
        double v = Kp * e + Ki * xI_candidate;

        // Saturation
        double u = v;
        if (u > uMax) u = uMax;
        if (u < uMin) u = uMin;

        // Clamping-based anti-windup:
        // If saturated and error would push further into saturation -> do not update integral
        boolean pushingUp = (u >= uMax) && (e > 0.0);
        boolean pushingDown = (u <= uMin) && (e < 0.0);
        if (!pushingUp && !pushingDown) {
            xI = xI_candidate;
        }

        return u;
    }

    public void reset() {
        xI = 0.0;
    }
}
