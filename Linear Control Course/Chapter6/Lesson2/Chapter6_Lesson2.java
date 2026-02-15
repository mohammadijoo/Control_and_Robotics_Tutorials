public final class SecondOrderSystem {

    private final double zeta;
    private final double omegaN;

    public SecondOrderSystem(double poleReal1, double poleImag1,
                             double poleReal2, double poleImag2) {
        // p1 = poleReal1 + j poleImag1
        // p2 = poleReal2 + j poleImag2
        double sumReal  = poleReal1 + poleReal2;
        double sumImag  = poleImag1 + poleImag2;
        double prodReal = poleReal1 * poleReal2 - poleImag1 * poleImag2;
        double prodImag = poleReal1 * poleImag2 + poleReal2 * poleImag1;

        // For a stable second-order system, the product should be real > 0
        if (Math.abs(prodImag) > 1e-6) {
            throw new IllegalArgumentException("Poles are not a valid conjugate pair.");
        }

        double omegaN = Math.sqrt(prodReal);
        double zeta   = -sumReal / (2.0 * omegaN);

        this.zeta   = zeta;
        this.omegaN = omegaN;
    }

    public double getZeta() {
        return zeta;
    }

    public double getOmegaN() {
        return omegaN;
    }

    public double[] canonicalDenominator() {
        // returns [1, 2 zeta omega_n, omega_n^2]
        return new double[] {1.0, 2.0 * zeta * omegaN, omegaN * omegaN};
    }
}
