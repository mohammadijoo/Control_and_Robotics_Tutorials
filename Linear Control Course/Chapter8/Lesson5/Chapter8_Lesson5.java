public class FirstOrderTradeOff {

    public static class Metrics {
        public double ess;
        public double ts;
        public double umax;
    }

    public static Metrics simulate(double K, double tFinal, double dt) {
        int nSteps = (int) (tFinal / dt);
        double[] y = new double[nSteps + 1];
        double[] u = new double[nSteps + 1];
        double[] t = new double[nSteps + 1];

        double x = 0.0;
        double r = 1.0;

        for (int k = 0; k <= nSteps; ++k) {
            t[k] = k * dt;
            double e = r - x;
            u[k] = K * e;
            y[k] = x;
            double xdot = -x + u[k];
            x += dt * xdot;
        }

        double yss = y[nSteps];
        double ess = r - yss;
        double tol = 0.02 * Math.abs(yss);
        double ts = tFinal;
        outer:
        for (int k = 0; k <= nSteps; ++k) {
            boolean ok = true;
            for (int j = k; j <= nSteps; ++j) {
                if (Math.abs(y[j] - yss) > tol) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                ts = t[k];
                break outer;
            }
        }

        double umax = 0.0;
        for (int k = 0; k <= nSteps; ++k) {
            umax = Math.max(umax, Math.abs(u[k]));
        }

        Metrics m = new Metrics();
        m.ess = ess;
        m.ts = ts;
        m.umax = umax;
        return m;
    }

    public static void main(String[] args) {
        double[] gains = {1.0, 2.0, 5.0, 10.0};
        for (double K : gains) {
            Metrics m = simulate(K, 10.0, 1e-3);
            System.out.printf("K = %.1f | ess = %.4f, ts = %.3f, umax = %.3f%n",
                              K, m.ess, m.ts, m.umax);
        }
    }
}
