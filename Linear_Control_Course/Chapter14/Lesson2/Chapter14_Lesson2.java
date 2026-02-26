public class BodeFactors {

    public static class BodePoint {
        public double omega;
        public double magDb;
        public double phaseDeg;
        public BodePoint(double omega, double magDb, double phaseDeg) {
            this.omega = omega;
            this.magDb = magDb;
            this.phaseDeg = phaseDeg;
        }
    }

    public static BodePoint firstOrderPole(double omega, double omegaC) {
        double x = omega / omegaC;
        double mag = 1.0 / Math.sqrt(1.0 + x * x);
        double magDb = 20.0 * Math.log10(mag);
        double phaseRad = -Math.atan(x);
        double phaseDeg = Math.toDegrees(phaseRad);
        return new BodePoint(omega, magDb, phaseDeg);
    }

    public static BodePoint secondOrderPole(double omega, double omegaN, double zeta) {
        double r = omega / omegaN;
        double real = 1.0 - r * r;
        double imag = 2.0 * zeta * r;
        double denomMag = Math.sqrt(real * real + imag * imag);
        double mag = 1.0 / denomMag;
        double magDb = 20.0 * Math.log10(mag);
        double phaseRad = -Math.atan2(imag, real);
        double phaseDeg = Math.toDegrees(phaseRad);
        return new BodePoint(omega, magDb, phaseDeg);
    }

    public static void main(String[] args) {
        double omegaC = 10.0;
        double omegaN = 20.0;
        double zeta = 0.5;

        double[] omegas = {0.1, 1.0, 10.0, 100.0, 1000.0};

        System.out.println("First-order pole:");
        for (double omega : omegas) {
            BodePoint bp = firstOrderPole(omega, omegaC);
            System.out.printf("omega=%.3f, |G| dB=%.3f, phase=%.3f deg%n",
                              bp.omega, bp.magDb, bp.phaseDeg);
        }

        System.out.println("\nSecond-order pole:");
        for (double omega : omegas) {
            BodePoint bp = secondOrderPole(omega, omegaN, zeta);
            System.out.printf("omega=%.3f, |G| dB=%.3f, phase=%.3f deg%n",
                              bp.omega, bp.magDb, bp.phaseDeg);
        }
    }
}
