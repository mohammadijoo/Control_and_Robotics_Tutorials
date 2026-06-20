public class FirstOrderLowPass {
    private final double tau;
    private double x;

    public FirstOrderLowPass(double tau, double x0) {
        this.tau = tau;
        this.x = x0;
    }

    public FirstOrderLowPass(double tau) {
        this(tau, 0.0);
    }

    /**
     * Update the filter state using forward-Euler integration.
     * @param u  current input sample
     * @param dt sampling period (seconds)
     * @return filtered output y_k
     */
    public double update(double u, double dt) {
        double a = -1.0 / tau;
        double b =  1.0 / tau;
        x = x + dt * (a * x + b * u);
        return x;
    }
}

// Example usage (e.g., in a Java-based robot controller such as WPILib robot code):
// FirstOrderLowPass lp = new FirstOrderLowPass(0.03);
// double yFilt = lp.update(encoderMeasurement, dt);
