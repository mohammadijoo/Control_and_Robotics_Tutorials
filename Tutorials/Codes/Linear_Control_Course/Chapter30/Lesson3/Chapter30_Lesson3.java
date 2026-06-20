public class SaturatedFirstOrder {
    private static double sat(double u, double uMax) {
        return Math.max(-uMax, Math.min(u, uMax));
    }

    public static void main(String[] args) {
        double k = 5.0;
        double D = 3.0;
        double uMax = 1.0;
        double x0 = 0.0;
        double tFinal = 20.0;
        double dt = 0.001;

        int N = (int) (tFinal / dt);
        double[] t = new double[N + 1];
        double[] x = new double[N + 1];
        double[] uCmd = new double[N + 1];
        double[] uApplied = new double[N + 1];

        x[0] = x0;
        t[0] = 0.0;

        for (int n = 0; n < N; ++n) {
            t[n + 1] = t[n] + dt;

            uCmd[n] = -k * x[n];
            uApplied[n] = sat(uCmd[n], uMax);

            double xDot = -x[n] + uApplied[n] + D;
            x[n + 1] = x[n] + dt * xDot;
        }

        uCmd[N] = -k * x[N];
        uApplied[N] = sat(uCmd[N], uMax);

        for (int n = 0; n <= N; n += N / 10) {
            System.out.printf("t=%.3f  x=%.4f  u=%.4f%n", t[n], x[n], uApplied[n]);
        }
    }
}
