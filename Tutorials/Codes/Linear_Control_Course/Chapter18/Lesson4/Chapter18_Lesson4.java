public final class SecondOrderDesign {

    private SecondOrderDesign() {}

    public static double zetaFromMpApprox(double Mp) {
        // crude approximation based on typical textbook values
        if (Mp <= 0.05)      return 0.7;
        else if (Mp <= 0.1) return 0.6;
        else if (Mp <= 0.2) return 0.5;
        else                 return 0.4;
    }

    public static double mrFromZeta(double zeta) {
        if (zeta >= 1.0 / Math.sqrt(2.0)) {
            return 1.0;
        }
        return 1.0 / (2.0 * zeta * Math.sqrt(1.0 - zeta*zeta));
    }

    public static double wnFromTs(double ts, double zeta) {
        return 4.0 / (zeta * ts);
    }

    public static double bandwidthFromSecondOrder(double wn, double zeta) {
        double term = Math.sqrt(2.0 - 4.0*zeta*zeta + 4.0*Math.pow(zeta, 4));
        double y = 1.0 - 2.0*zeta*zeta + term;
        return wn * Math.sqrt(y);
    }

    public static void main(String[] args) {
        double MpMax = 0.1; // 10% overshoot
        double tsMax = 1.0; // 1 s

        double zeta = zetaFromMpApprox(MpMax);
        double wn   = wnFromTs(tsMax, zeta);
        double wB   = bandwidthFromSecondOrder(wn, zeta);
        double Mr   = mrFromZeta(zeta);

        System.out.println("zeta ~ " + zeta);
        System.out.println("wn   ~ " + wn);
        System.out.println("omega_B ~ " + wB);
        System.out.println("Mr   ~ " + Mr);

        // In a robot axis controller, these metrics can be logged
        // and compared against target bandwidth and overshoot limits.
    }
}
