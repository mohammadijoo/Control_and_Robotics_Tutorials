
public final class Metrics {

    public static double computeISE(double[] t, double[][] e) {
        int N = t.length;
        if (N < 2 || e.length != N) {
            throw new IllegalArgumentException("Invalid log lengths");
        }
        double J = 0.0;
        for (int k = 1; k < N; ++k) {
            double dt = t[k] - t[k - 1];
            double norm2 = 0.0;
            for (int i = 0; i < e[k - 1].length; ++i) {
                double v = e[k - 1][i];
                norm2 += v * v;
            }
            J += norm2 * dt;
        }
        return J;
    }

    public static double[] computeRMS(double[] t, double[][] e) {
        int N = t.length;
        int n = e[0].length;
        double[] sqInt = new double[n];
        double T = 0.0;

        for (int k = 1; k < N; ++k) {
            double dt = t[k] - t[k - 1];
            for (int i = 0; i < n; ++i) {
                double v = e[k - 1][i];
                sqInt[i] += dt * v * v;
            }
            T += dt;
        }

        double[] rms = new double[n];
        for (int i = 0; i < n; ++i) {
            rms[i] = Math.sqrt(sqInt[i] / T);
        }
        return rms;
    }
}
