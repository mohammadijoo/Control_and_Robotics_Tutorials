public final class ValidationMetricsJava {

    public static class Metrics {
        public final double rmse;
        public final double vaf;
        public final double fitPercent;
        public final double r2;
        public final double aic;
        public final double bic;

        public Metrics(double rmse, double vaf, double fitPercent,
                       double r2, double aic, double bic) {
            this.rmse = rmse;
            this.vaf = vaf;
            this.fitPercent = fitPercent;
            this.r2 = r2;
            this.aic = aic;
            this.bic = bic;
        }
    }

    public static Metrics compute(double[] tauMeas, double[] tauPred, int pParams) {
        int N = tauMeas.length;
        if (N != tauPred.length) {
            throw new IllegalArgumentException("Array sizes must match");
        }

        double[] e = new double[N];
        double sumESq = 0.0;
        double sumTau = 0.0;

        for (int k = 0; k < N; ++k) {
            e[k] = tauMeas[k] - tauPred[k];
            sumESq += e[k] * e[k];
            sumTau += tauMeas[k];
        }

        double rmse = Math.sqrt(sumESq / N);
        double tauMean = sumTau / N;

        double tss = 0.0;
        for (int k = 0; k < N; ++k) {
            double d = tauMeas[k] - tauMean;
            tss += d * d;
        }

        // Variances
        double eMean = 0.0;
        for (int k = 0; k < N; ++k) {
            eMean += e[k];
        }
        eMean /= N;

        double varY = 0.0;
        double varE = 0.0;
        for (int k = 0; k < N; ++k) {
            double dy = tauMeas[k] - tauMean;
            double de = e[k] - eMean;
            varY += dy * dy;
            varE += de * de;
        }
        varY /= (N - 1);
        varE /= (N - 1);

        double vaf = 100.0 * (1.0 - varE / varY);

        double num = 0.0;
        double den = 0.0;
        for (int k = 0; k < N; ++k) {
            double d1 = tauMeas[k] - tauPred[k];
            double d2 = tauMeas[k] - tauMean;
            num += d1 * d1;
            den += d2 * d2;
        }
        num = Math.sqrt(num);
        den = Math.sqrt(den);
        double fitPercent = 100.0 * (1.0 - num / den);

        double rss = sumESq;
        double r2 = 1.0 - rss / tss;

        double sigma2Hat = rss / N;
        double aic = 2.0 * pParams + N * Math.log(sigma2Hat);
        double bic = pParams * Math.log(N) + N * Math.log(sigma2Hat);

        return new Metrics(rmse, vaf, fitPercent, r2, aic, bic);
    }

    private ValidationMetricsJava() {
        // utility
    }
}
      
