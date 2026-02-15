public class HriMetrics {

    public static double shannonID(double D, double W) {
        return (Math.log(1.0 + D / W) / Math.log(2.0));
    }

    public static double fittsMT(double D, double W, double a, double b) {
        return a + b * shannonID(D, W);
    }

    public static double throughput(double[] IDs, double[] MTs) {
        double sumID = 0.0;
        double sumMT = 0.0;
        for (int i = 0; i < IDs.length; ++i) {
            sumID += IDs[i];
            sumMT += MTs[i];
        }
        return sumID / sumMT;
    }

    public static double safeVelocity(double t_r, double a_max, double d_safe) {
        // Solve v^2/(2 a_max) + t_r v - d_safe = 0 for v >= 0.
        double A = 1.0 / (2.0 * a_max);
        double B = t_r;
        double C = -d_safe;
        double disc = B*B - 4.0*A*C;
        if (disc < 0.0) return 0.0;
        double sqrt_disc = Math.sqrt(disc);
        double v1 = (-B + sqrt_disc) / (2.0 * A);
        double v2 = (-B - sqrt_disc) / (2.0 * A);
        double v = Double.POSITIVE_INFINITY;
        if (v1 >= 0.0 && v1 < v) v = v1;
        if (v2 >= 0.0 && v2 < v) v = v2;
        if (Double.isInfinite(v)) v = 0.0;
        return v;
    }

    public static void main(String[] args) {
        double[] IDs = new double[] {
            shannonID(0.2, 0.04),
            shannonID(0.3, 0.05),
            shannonID(0.4, 0.06)
        };
        double[] MTs = new double[] {0.45, 0.50, 0.55};
        double TP = throughput(IDs, MTs);

        double t_r = 0.4;
        double a_max = 3.0;
        double d_safe = 1.5;
        double v_max = safeVelocity(t_r, a_max, d_safe);

        // In an Android or Java-based HRI dashboard, display TP and v_max.
    }
}
      
