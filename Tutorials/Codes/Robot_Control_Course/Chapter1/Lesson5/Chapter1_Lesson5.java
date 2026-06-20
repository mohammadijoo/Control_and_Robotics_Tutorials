
public class SecondOrderStepMetrics {

    public static class Specs {
        public double zeta;
        public double omegaN;
        public Specs(double zeta, double omegaN) {
            this.zeta = zeta;
            this.omegaN = omegaN;
        }
    }

    public static class Metrics {
        public double Mp;
        public double Ts;
        public double Tr;
        public double ess;
    }

    public static Metrics compute(Specs s, double dt, double tFinal) {
        int n = (int) (tFinal / dt);
        double[] t = new double[n];
        double[] y = new double[n];
        double[] r = new double[n];

        // Build transfer function in discrete-time via simple Euler approximation
        double zeta = s.zeta;
        double wn = s.omegaN;

        double yk = 0.0, ydotk = 0.0;
        double yInf = 1.0;
        for (int k = 0; k < n; ++k) {
            t[k] = k * dt;
            r[k] = 1.0;
            double yddot = -2.0 * zeta * wn * ydotk - wn * wn * yk + wn * wn * r[k];
            ydotk += dt * yddot;
            yk += dt * ydotk;
            y[k] = yk;
        }

        Metrics m = new Metrics();

        // Mp
        double yMax = y[0];
        for (int k = 1; k < n; ++k) {
            if (y[k] > yMax) yMax = y[k];
        }
        m.Mp = 100.0 * Math.max(0.0, (yMax - yInf) / Math.abs(yInf));

        // Ts (2 percent)
        double tol = 0.02 * Math.abs(yInf);
        m.Ts = t[n - 1];
        outer:
        for (int k = 0; k < n; ++k) {
            boolean ok = true;
            for (int j = k; j < n; ++j) {
                if (Math.abs(y[j] - yInf) > tol) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                m.Ts = t[k];
                break outer;
            }
        }

        // ess
        m.ess = r[n - 1] - y[n - 1];

        // Simple rise time 10-90 percent
        double y10 = 0.1 * yInf;
        double y90 = 0.9 * yInf;
        double t10 = t[0];
        double t90 = t[n - 1];
        boolean found10 = false, found90 = false;
        for (int k = 0; k < n; ++k) {
            if (!found10 && y[k] >= y10) {
                t10 = t[k];
                found10 = true;
            }
            if (!found90 && y[k] >= y90) {
                t90 = t[k];
                found90 = true;
            }
        }
        m.Tr = t90 - t10;

        return m;
    }

    public static void main(String[] args) {
        Specs s = new Specs(0.7, 8.0);
        Metrics m = compute(s, 1e-3, 5.0);
        System.out.println("Mp = " + m.Mp + " percent");
        System.out.println("Ts = " + m.Ts + " s");
        System.out.println("Tr = " + m.Tr + " s");
        System.out.println("ess = " + m.ess);
    }
}
