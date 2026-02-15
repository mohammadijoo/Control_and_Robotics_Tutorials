public class RootLocusGainSelector {

    // For G(s) = 1 / (s (s + 2)), closed-loop polynomial is
    // s^2 + 2 s + K = 0. Poles: s = -1 ± sqrt(1 - K) or -1 ± j sqrt(K - 1).
    // We compute zeta(K) = 1 / sqrt(K) for K > 1 and search for zeta >= zetaMin.

    public static double dampingRatioFromK(double K) {
        if (K <= 1.0) {
            return 1.0; // overdamped (zeta >= 1)
        }
        return 1.0 / Math.sqrt(K);
    }

    public static double selectGainForZeta(double zetaMin,
                                           double Kmin,
                                           double Kmax,
                                           double dK) {
        double bestK = -1.0;
        for (double K = Kmin; K <= Kmax; K += dK) {
            double zeta = dampingRatioFromK(K);
            if (zeta >= zetaMin && K > 1.0) {
                bestK = K;
                break;
            }
        }
        return bestK;
    }

    public static void main(String[] args) {
        double zetaMin = 0.6;  // e.g., ≈ 10% overshoot requirement
        double Kmin    = 1.01;
        double Kmax    = 20.0;
        double dK      = 0.001;

        double Kstar = selectGainForZeta(zetaMin, Kmin, Kmax, dK);
        if (Kstar > 0.0) {
            System.out.println("Selected gain K* ≈ " + Kstar);
            double zeta = dampingRatioFromK(Kstar);
            double Ts   = 4.0; // as before
            System.out.println("Damping ratio zeta ≈ " + zeta);
            System.out.println("Approximate Ts(2%) ≈ " + Ts + " s");
        } else {
            System.out.println("No suitable gain found in the search interval.");
        }
    }
}
