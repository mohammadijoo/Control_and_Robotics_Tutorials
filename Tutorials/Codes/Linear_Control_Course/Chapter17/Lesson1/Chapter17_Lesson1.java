import org.apache.commons.math3.complex.Complex;

public class GainPhaseMargins {

    public static Complex L(Complex s, double K) {
        // L(s) = K / (s (s + 1))
        return new Complex(K, 0.0).divide(s.multiply(s.add(new Complex(1.0, 0.0))));
    }

    public static void main(String[] args) {
        double K = 1.0;
        int N = 2000;
        double wMin = 1e-2;
        double wMax = 1e2;

        double prevMagDb = 0.0;
        double prevPhaseDeg = 0.0;
        boolean first = true;
        double wGc = 0.0;
        boolean foundGc = false;

        for (int i = 0; i < N; ++i) {
            double alpha = (double) i / (double) (N - 1);
            double w = wMin * Math.pow(wMax / wMin, alpha);

            Complex s = new Complex(0.0, w);
            Complex Lval = L(s, K);
            double mag = Lval.abs();
            double phaseDeg = Math.toDegrees(Math.atan2(Lval.getImaginary(), Lval.getReal()));
            double magDb = 20.0 * Math.log10(mag);

            if (!first) {
                if ((prevMagDb > 0.0 && magDb <= 0.0) ||
                    (prevMagDb < 0.0 && magDb >= 0.0)) {
                    wGc = w;
                    foundGc = true;
                    break;
                }
            }
            prevMagDb = magDb;
            prevPhaseDeg = phaseDeg;
            first = false;
        }

        if (foundGc) {
            Complex sGc = new Complex(0.0, wGc);
            Complex Lgc = L(sGc, K);
            double phaseDegGc =
                Math.toDegrees(Math.atan2(Lgc.getImaginary(), Lgc.getReal()));
            double phiM = 180.0 + phaseDegGc;
            System.out.println("Approximate phase margin (deg): " + phiM);
        } else {
            System.out.println("No gain crossover found.");
        }

        // In a WPILib robot project, similar computations can be done offline
        // to verify that position or velocity PID loops have sufficient margins.
    }
}
