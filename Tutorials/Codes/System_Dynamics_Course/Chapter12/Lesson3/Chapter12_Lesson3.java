// Chapter12_Lesson3.java
// System Dynamics — Chapter 12, Lesson 3
// Resonance, Bandwidth, and Quality Factor in Mechanical and Electrical Systems
//
// Compile:
//   javac Chapter12_Lesson3.java
// Run:
//   java Chapter12_Lesson3
//
// Notes:
// - This file avoids external dependencies by using closed-form magnitude formulas.
// - For richer numerical computing (complex arithmetic, fitting), consider Apache Commons Math.

public class Chapter12_Lesson3 {

    static class SecondOrderParams {
        final double wn;   // rad/s
        final double zeta; // damping ratio
        final double Q;    // quality factor

        SecondOrderParams(double wn, double zeta, double Q) {
            this.wn = wn; this.zeta = zeta; this.Q = Q;
        }
    }

    static SecondOrderParams secondOrderFromMCK(double m, double c, double k) {
        if (m <= 0.0 || k <= 0.0) throw new IllegalArgumentException("m and k must be positive.");
        double wn = Math.sqrt(k / m);
        double zeta = c / (2.0 * Math.sqrt(k * m));
        double Q = (zeta <= 0.0) ? Double.POSITIVE_INFINITY : 1.0 / (2.0 * zeta);
        return new SecondOrderParams(wn, zeta, Q);
    }

    static Double resonanceFrequency(double wn, double zeta) {
        double crit = 1.0 / Math.sqrt(2.0);
        if (zeta >= crit) return null;
        return wn * Math.sqrt(1.0 - 2.0 * zeta * zeta);
    }

    static double[] halfPowerFrequencies(double wn, double zeta) {
        Double wr = resonanceFrequency(wn, zeta);
        if (wr == null) return null;
        double inside = 1.0 - zeta * zeta;
        if (inside <= 0.0) return null;

        double r1sq = 1.0 - 2.0*zeta*zeta - 2.0*zeta*Math.sqrt(inside);
        double r2sq = 1.0 - 2.0*zeta*zeta + 2.0*zeta*Math.sqrt(inside);
        if (r1sq <= 0.0 || r2sq <= 0.0) return null;

        return new double[] { wn*Math.sqrt(r1sq), wn*Math.sqrt(r2sq) };
    }

    // Normalized low-pass magnitude squared:
    // |G(jw)|^2 = 1 / ((1-r^2)^2 + (2 zeta r)^2), r = w/wn
    static double magNormalizedLowpass(double w, double wn, double zeta) {
        double r = w / wn;
        double denom = (1.0 - r*r)*(1.0 - r*r) + (2.0*zeta*r)*(2.0*zeta*r);
        return 1.0 / Math.sqrt(denom);
    }

    public static void main(String[] args) {
        double m = 1.0, c = 0.4, k = 100.0;
        SecondOrderParams p = secondOrderFromMCK(m, c, k);

        System.out.println("Mass–spring–damper parameters:");
        System.out.println("  wn   = " + p.wn + " rad/s");
        System.out.println("  zeta = " + p.zeta);
        System.out.println("  Q    = " + p.Q);

        Double wr = resonanceFrequency(p.wn, p.zeta);
        if (wr != null) {
            System.out.println("  wr   = " + wr + " rad/s");
            double[] hp = halfPowerFrequencies(p.wn, p.zeta);
            if (hp != null) {
                double w1 = hp[0], w2 = hp[1];
                double bw = w2 - w1;
                System.out.println("  w1   = " + w1 + " rad/s");
                System.out.println("  w2   = " + w2 + " rad/s");
                System.out.println("  BW   = " + bw + " rad/s");
                System.out.println("  Q_hp = " + (wr / bw) + " (wr/BW)");
            }
        } else {
            System.out.println("  wr   = (no resonant peak; zeta >= 1/sqrt(2))");
        }

        // Series RLC
        double R = 10.0, L = 50e-3, C = 10e-6;
        double w0 = 1.0 / Math.sqrt(L*C);
        double Q = (w0 * L) / R;
        double BW = R / L;
        System.out.println("\nSeries RLC:");
        System.out.println("  w0 = " + w0 + " rad/s");
        System.out.println("  Q  = " + Q);
        System.out.println("  BW = " + BW + " rad/s");
    }
}
