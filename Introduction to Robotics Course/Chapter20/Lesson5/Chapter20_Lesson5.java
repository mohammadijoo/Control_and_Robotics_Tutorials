public class Metrics {

    public static double computeISE(double[] t, double[] r, double[] y) {
        int N = t.length;
        if (N == 0) return 0.0;
        double J = 0.0;
        for (int k = 0; k + 1 < N; ++k) {
            double dt = t[k + 1] - t[k];
            double e = r[k] - y[k];
            J += e * e * dt;
        }
        return J;
    }

    public static double computeRMS(double[] r, double[] y) {
        int N = r.length;
        if (N == 0) return 0.0;
        double sumSq = 0.0;
        for (int k = 0; k < N; ++k) {
            double e = r[k] - y[k];
            sumSq += e * e;
        }
        return Math.sqrt(sumSq / (double) N);
    }

    public static void main(String[] args) {
        double[] t = {0.0, 0.01, 0.02};
        double[] r = {1.0, 1.0, 1.0};
        double[] y = {0.8, 0.9, 0.95};

        double J_ISE = computeISE(t, r, y);
        double e_RMS = computeRMS(r, y);

        System.out.println("ISE = " + J_ISE);
        System.out.println("RMS error = " + e_RMS);
    }
}
      
