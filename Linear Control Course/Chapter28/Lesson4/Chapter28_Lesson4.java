public class PDSweep {

    static class Metrics {
        final double zeta;
        final double wn;
        final boolean stable;
        Metrics(double zeta, double wn, boolean stable) {
            this.zeta = zeta;
            this.wn = wn;
            this.stable = stable;
        }
    }

    static Metrics metricsFromParams(double J, double B, double Kp, double Kd) {
        boolean stable = (J > 0.0) && (B + Kd > 0.0) && (Kp > 0.0);
        double wn = Math.sqrt(Kp / J);
        double zeta = (B + Kd) / (2.0 * Math.sqrt(J * Kp));
        return new Metrics(zeta, wn, stable);
    }

    static double overshoot(double zeta) {
        if (zeta <= 0.0 || zeta >= 1.0) return 0.0;
        return Math.exp(-Math.PI * zeta / Math.sqrt(1.0 - zeta * zeta));
    }

    static double settlingTime(double zeta, double wn) {
        return 4.0 / (zeta * wn);
    }

    public static void main(String[] args) {
        double[] Jvals  = {0.8, 1.0, 1.2};
        double[] Bvals  = {0.3, 0.5, 0.7};
        double[] Kpvals = {10.0, 20.0, 30.0, 40.0};
        double[] Kdvals = {0.5, 1.0, 1.5, 2.0};

        double zetaMin = 0.5;
        double MpMax   = 0.15;
        double tsMax   = 2.0;

        int total   = 0;
        int stableN = 0;
        int robustN = 0;

        for (double J : Jvals) {
            for (double B : Bvals) {
                for (double Kp : Kpvals) {
                    for (double Kd : Kdvals) {
                        total++;
                        Metrics m = metricsFromParams(J, B, Kp, Kd);
                        if (!m.stable) continue;
                        stableN++;

                        double Mp = overshoot(m.zeta);
                        double ts = settlingTime(m.zeta, m.wn);

                        if (m.zeta >= zetaMin && Mp <= MpMax && ts <= tsMax) {
                            robustN++;
                        }
                    }
                }
            }
        }

        System.out.println("Stable fraction: " + (double) stableN / total);
        System.out.println("Robust fraction: " + (double) robustN / total);
    }
}
