public class LowPassFilter {
    private final double tau;
    private final double dt;
    private final double alpha;
    private double y;

    public LowPassFilter(double tau, double dt, double initialValue) {
        this.tau = tau;
        this.dt = dt;
        this.alpha = Math.exp(-dt / tau);
        this.y = initialValue;
    }

    public double update(double yMeas) {
        y = alpha * y + (1.0 - alpha) * yMeas;
        return y;
    }

    public void reset(double initialValue) {
        y = initialValue;
    }

    public static void main(String[] args) {
        double dt = 0.02;   // 50 Hz control loop
        double tau = 0.1;   // filter time constant
        LowPassFilter filter = new LowPassFilter(tau, dt, 0.0);

        double[] yMeas = {0.0, 0.8, 1.2, 0.9, 1.1};
        for (int k = 0; k < yMeas.length; ++k) {
            double yf = filter.update(yMeas[k]);
            System.out.println("k=" + k
                + " y_meas=" + yMeas[k]
                + " y_filt=" + yf);
        }
    }
}
