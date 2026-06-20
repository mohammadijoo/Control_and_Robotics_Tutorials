public final class ConfigMetric {
    private final double[] weights;
    private final boolean[] revolute;

    public ConfigMetric(double[] weights, boolean[] revolute) {
        if (weights.length != revolute.length) {
            throw new IllegalArgumentException("Dimension mismatch");
        }
        this.weights = weights.clone();
        this.revolute = revolute.clone();
    }

    private static double wrapAngle(double angle) {
        double pi = Math.PI;
        double a = (angle + pi) % (2.0 * pi);
        if (a < 0.0) {
            a += 2.0 * pi;
        }
        return a - pi;
    }

    public double distance(double[] q1, double[] q2) {
        if (q1.length != q2.length || q1.length != weights.length) {
            throw new IllegalArgumentException("Dimension mismatch");
        }
        double sum = 0.0;
        for (int i = 0; i < q1.length; ++i) {
            double diff = q2[i] - q1[i];
            if (revolute[i]) {
                diff = wrapAngle(diff);
            }
            sum += weights[i] * diff * diff;
        }
        return Math.sqrt(sum);
    }
}
      
