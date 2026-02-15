import java.util.ArrayList;
import java.util.List;

public class SecondOrderMetrics {

    public static double[] stepResponse(double[] t, double zeta, double wn) {
        double sqrtTerm = Math.sqrt(1.0 - zeta * zeta);
        double wd = wn * sqrtTerm;
        double phi = Math.atan2(sqrtTerm, zeta);
        double[] y = new double[t.length];
        for (int i = 0; i < t.length; ++i) {
            double tk = t[i];
            double envelope = Math.exp(-zeta * wn * tk);
            double s = Math.sin(wd * tk + phi);
            y[i] = 1.0 - (envelope / sqrtTerm) * s;
        }
        return y;
    }

    public static class Metrics {
        public double tr;
        public double tp;
        public double Mp;
        public double ts;
        public double cInf;
    }

    public static Metrics metricsFromSamples(double[] t, double[] y, double band) {
        int n = t.length;
        int nTail = Math.max(1, n / 10);

        // Final value
        double cInf = 0.0;
        for (int i = n - nTail; i < n; ++i) {
            cInf += y[i];
        }
        cInf /= (double) nTail;

        double low = 0.1 * cInf;
        double high = 0.9 * cInf;

        Double t10 = null;
        Double t90 = null;
        for (int i = 0; i < n; ++i) {
            if (t10 == null && y[i] >= low) {
                t10 = t[i];
            }
            if (t90 == null && y[i] >= high) {
                t90 = t[i];
                break;
            }
        }
        double tr = (t10 != null && t90 != null) ? (t90 - t10) : -1.0;

        // Peak
        int idxMax = 0;
        for (int i = 1; i < n; ++i) {
            if (y[i] > y[idxMax]) {
                idxMax = i;
            }
        }
        double tp = t[idxMax];
        double Mp = (y[idxMax] - cInf) / cInf;

        // Settling time
        double tol = band * Math.abs(cInf);
        double ts = t[n - 1];
        outer:
        for (int k = 0; k < n; ++k) {
            for (int j = k; j < n; ++j) {
                if (Math.abs(y[j] - cInf) > tol) {
                    continue outer;
                }
            }
            ts = t[k];
            break;
        }

        Metrics m = new Metrics();
        m.tr = tr;
        m.tp = tp;
        m.Mp = Mp;
        m.ts = ts;
        m.cInf = cInf;
        return m;
    }

    public static void main(String[] args) {
        double zeta = 0.6;
        double wn = 12.0;
        double dt = 0.001;
        double tEnd = 3.0;
        List<Double> tList = new ArrayList<>();
        for (double tk = 0.0; tk <= tEnd + 1e-12; tk += dt) {
            tList.add(tk);
        }
        double[] t = new double[tList.size()];
        for (int i = 0; i < t.length; ++i) {
            t[i] = tList.get(i);
        }
        double[] y = stepResponse(t, zeta, wn);
        Metrics m = metricsFromSamples(t, y, 0.02);

        System.out.println("tr = " + m.tr + ", tp = " + m.tp
                + ", Mp = " + m.Mp + ", ts = " + m.ts);
    }
}
