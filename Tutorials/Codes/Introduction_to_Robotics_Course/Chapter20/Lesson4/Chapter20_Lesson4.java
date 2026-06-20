public final class Metrics {

    public static class Result {
        public double iae;
        public double ise;
        public double itae;
        public double rmse;
        public double maxAbsErr;
    }

    public static Result compute(double[] t, double[] r, double[] y) {
        if (t.length == 0 || r.length != t.length || y.length != t.length) {
            throw new IllegalArgumentException("Arrays must have same nonzero length.");
        }

        int N = t.length;
        double dt = (N > 1) ? (t[1] - t[0]) : 0.0;

        double iae = 0.0;
        double ise = 0.0;
        double itae = 0.0;
        double sumE2 = 0.0;
        double maxAbsErr = 0.0;

        for (int k = 0; k < N; ++k) {
            double e = r[k] - y[k];
            double ae = Math.abs(e);

            iae += ae * dt;
            ise += e * e * dt;
            itae += t[k] * ae * dt;
            sumE2 += e * e;

            if (ae > maxAbsErr) {
                maxAbsErr = ae;
            }
        }

        Result res = new Result();
        res.iae = iae;
        res.ise = ise;
        res.itae = itae;
        res.rmse = Math.sqrt(sumE2 / (double) N);
        res.maxAbsErr = maxAbsErr;
        return res;
    }

    public static void main(String[] args) {
        int N = 1001;
        double T = 10.0;
        double[] t = new double[N];
        double[] r = new double[N];
        double[] y = new double[N];

        for (int k = 0; k < N; ++k) {
            double tk = T * (double) k / (double) (N - 1);
            t[k] = tk;
            r[k] = 1.0;
            y[k] = 1.0 - Math.exp(-tk) * Math.cos(2.0 * tk);
        }

        Result res = compute(t, r, y);
        System.out.println("RMSE = " + res.rmse);
        System.out.println("max |e| = " + res.maxAbsErr);
    }
}
      
