public class SecondOrderAnalysis {

    public static class Result {
        public double Mr;
        public double wr;
        public double wb;
    }

    public static Result analyze(double wn, double zeta) {
        int N = 2000;
        double wMin = 0.1;
        double wMax = 100.0;
        double[] w = new double[N];
        double[] mag = new double[N];

        for (int k = 0; k < N; ++k) {
            double alpha = (double) k / (double) (N - 1);
            w[k] = wMin * Math.pow(wMax / wMin, alpha);
            double x = w[k] / wn;
            double denom = Math.pow(1.0 - x * x, 2.0)
                         + Math.pow(2.0 * zeta * x, 2.0);
            mag[k] = 1.0 / Math.sqrt(denom);
        }

        // Resonant peak
        double Mr = mag[0];
        double wr = w[0];
        for (int k = 1; k < N; ++k) {
            if (mag[k] > Mr) {
                Mr = mag[k];
                wr = w[k];
            }
        }

        // -3 dB bandwidth
        double target = mag[0] / Math.sqrt(2.0);
        double wb = w[N - 1];
        for (int k = 0; k < N; ++k) {
            if (mag[k] <= target) {
                wb = w[k];
                break;
            }
        }

        Result r = new Result();
        r.Mr = Mr;
        r.wr = wr;
        r.wb = wb;
        return r;
    }

    public static void main(String[] args) {
        double wn = 10.0;
        double zeta = 0.5;

        Result r = analyze(wn, zeta);
        System.out.println("Mr = " + r.Mr
                + ", wr = " + r.wr + " rad/s"
                + ", wb = " + r.wb + " rad/s");

        // In a robot codebase (e.g. using WPILib), this could be distributed
        // as an offline tool to check servo bandwidth and resonance.
    }
}
