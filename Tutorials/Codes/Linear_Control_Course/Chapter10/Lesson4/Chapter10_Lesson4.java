public class RootLocusShaping {

    static class Specs {
        double zetaMin;
        double TsMax;
        Specs(double zetaMin, double TsMax) {
            this.zetaMin = zetaMin;
            this.TsMax = TsMax;
        }
    }

    static boolean specsSatisfied(double a, double K, Specs specs) {
        double disc = a * a - 4.0 * K;
        if (disc >= 0.0) {
            // poles are real; here we require underdamped behavior
            return false;
        }
        double sigma = -0.5 * a;
        double omega = 0.5 * Math.sqrt(-disc);
        double wn = Math.sqrt(sigma * sigma + omega * omega);
        if (wn == 0.0) return false;

        double zeta = -sigma / wn;
        double Ts = 4.0 / (-sigma);

        return (zeta >= specs.zetaMin) && (Ts <= specs.TsMax);
    }

    public static void main(String[] args) {
        double a = 4.0;
        Specs specs = new Specs(0.6, 1.0);

        double Kmin = 0.1, Kmax = 20.0;
        int N = 200;
        for (int i = 0; i <= N; ++i) {
            double K = Kmin + (Kmax - Kmin) * i / (double) N;
            if (specsSatisfied(a, K, specs)) {
                System.out.println("Acceptable gain K = " + K);
                break;
            }
        }
    }
}
