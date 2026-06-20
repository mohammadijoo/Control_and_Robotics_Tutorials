import java.util.ArrayList;
import java.util.List;

public class CSpace {
    private final double[] lower;
    private final double[] upper;
    private final boolean[] periodic;
    private final int n;

    public CSpace(double[] lowerBounds, double[] upperBounds, boolean[] periodicFlags) {
        if (lowerBounds.length != upperBounds.length ||
            lowerBounds.length != periodicFlags.length) {
            throw new IllegalArgumentException("Mismatched array lengths");
        }
        this.lower = lowerBounds.clone();
        this.upper = upperBounds.clone();
        this.periodic = periodicFlags.clone();
        this.n = lower.length;
    }

    private double[] wrap(double[] q) {
        double[] res = q.clone();
        for (int i = 0; i < n; ++i) {
            if (periodic[i]) {
                double x = res[i];
                // wrap to (-pi, pi]
                x = (x + Math.PI) % (2.0 * Math.PI);
                if (x < 0.0) {
                    x += 2.0 * Math.PI;
                }
                res[i] = x - Math.PI;
            }
        }
        return res;
    }

    private double[] shortestDifference(double[] qFrom, double[] qTo) {
        double[] a = wrap(qFrom);
        double[] b = wrap(qTo);
        double[] diff = new double[n];
        for (int i = 0; i < n; ++i) {
            double d = b[i] - a[i];
            if (periodic[i]) {
                if (d > Math.PI) {
                    d -= 2.0 * Math.PI;
                } else if (d < -Math.PI) {
                    d += 2.0 * Math.PI;
                }
            }
            diff[i] = d;
        }
        return diff;
    }

    public List<double[]> interpolate(double[] qStart,
                                        double[] qGoal,
                                        int numSamples) {
        double[] qs = wrap(qStart);
        double[] diff = shortestDifference(qs, qGoal);
        List<double[]> path = new ArrayList<>(numSamples);
        for (int k = 0; k < numSamples; ++k) {
            double alpha = (numSamples > 1)
                    ? (double) k / (double) (numSamples - 1)
                    : 0.0;
            double[] q = new double[n];
            for (int i = 0; i < n; ++i) {
                q[i] = qs[i] + alpha * diff[i];
            }
            q = wrap(q);
            path.add(q);
        }
        return path;
    }

    // Example usage in main:
    // public static void main(String[] args) {
    //     CSpace cspace = new CSpace(
    //         new double[]{-Math.PI, -Math.PI},
    //         new double[]{ Math.PI,  Math.PI},
    //         new boolean[]{true, true}
    //     );
    //     List<double[]> path = cspace.interpolate(
    //         new double[]{0.0, 0.0},
    //         new double[]{Math.PI, -Math.PI},
    //         5
    //     );
    // }
}
      
