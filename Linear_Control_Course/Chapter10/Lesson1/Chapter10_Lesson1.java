public final class TimeSpecUtils {

    public static double zetaFromMp(double MpPercent) {
        double Mp = MpPercent / 100.0;
        if (Mp <= 0.0 || Mp >= 1.0) {
            throw new IllegalArgumentException("MpPercent must be between 0 and 100.");
        }
        double lnMp = Math.log(Mp);
        return -lnMp / Math.sqrt(Math.PI * Math.PI + lnMp * lnMp);
    }

    public static double sigmaFromTs(double Ts, double perc) {
        if (Ts <= 0.0) {
            throw new IllegalArgumentException("Ts must be positive.");
        }
        if (Math.abs(perc - 2.0) < 1e-6) {
            return -4.0 / Ts;
        } else if (Math.abs(perc - 5.0) < 1e-6) {
            return -3.0 / Ts;
        } else {
            double eps = perc / 100.0;
            return Math.log(eps) / Ts; // negative
        }
    }

    public static boolean checkPole(double sigma, double omegaD,
                                    double MpPercent, double Ts) {
        if (sigma >= 0.0) return false;

        double omegaN = Math.sqrt(sigma * sigma + omegaD * omegaD);
        double zeta = -sigma / omegaN;

        double zetaMin = zetaFromMp(MpPercent);
        double sigmaMax = sigmaFromTs(Ts, 2.0);

        return (zeta >= zetaMin) && (sigma <= sigmaMax);
    }

    public static void main(String[] args) {
        double sigma = -3.0;
        double omegaD = 4.0;
        boolean ok = checkPole(sigma, omegaD, 10.0, 1.5);
        System.out.println("Pole satisfies specs? " + ok);
    }
}
