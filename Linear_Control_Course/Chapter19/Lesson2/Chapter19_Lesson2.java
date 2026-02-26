public class LagCompensator {
    private final double Kc;
    private final double beta;
    private final double T;
    private final double h;
    private final double a1;
    private final double b0;
    private final double b1;
    private double uPrev = 0.0;
    private double ePrev = 0.0;

    public LagCompensator(double Kc, double beta, double T, double h) {
        this.Kc = Kc;
        this.beta = beta;
        this.T = T;
        this.h = h;
        double denom = beta * T / h + 1.0;
        this.a1 = (beta * T / h) / denom;
        this.b0 = (Kc * (T / h + 1.0)) / denom;
        this.b1 = (-Kc * (T / h)) / denom;
    }

    public double update(double error) {
        double u = a1 * uPrev + b0 * error + b1 * ePrev;
        uPrev = u;
        ePrev = error;
        return u;
    }
}

// Example usage in a robot periodic loop (e.g., teleopPeriodic):
// LagCompensator lag = new LagCompensator(5.0, 5.0, 1.0, 0.02);
// double error = referencePosition - measuredPosition;
// double control = lag.update(error);
