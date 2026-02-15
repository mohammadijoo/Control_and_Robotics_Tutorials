public class LowPassFilter {
    private final double alpha;
    private double yPrev = 0.0;

    public LowPassFilter(double dt, double tau) {
        this.alpha = dt / (tau + dt);
    }

    public double step(double x) {
        double y = alpha * x + (1.0 - alpha) * yPrev;
        yPrev = y;
        return y;
    }

    public static void main(String[] args) {
        LowPassFilter lpf = new LowPassFilter(0.005, 0.08);
        double[] samples = {0.0, 0.1, 0.3, 0.2, 0.5, 0.4};
        for (double s : samples) {
          System.out.println(lpf.step(s));
        }
    }
}
      