public class LeadLagCompensator {
    private final double K;
    private final double z1, p1;
    private final double z2, p2;

    public LeadLagCompensator(double K, double z1, double p1, double z2, double p2) {
        this.K = K;
        this.z1 = z1;
        this.p1 = p1;
        this.z2 = z2;
        this.p2 = p2;
    }

    public double magnitude(double w) {
        // |(j w + z1)(j w + z2)/(j w + p1)(j w + p2)| = K * sqrt(...)
        double w2 = w * w;
        double num = (w2 + z1 * z1) * (w2 + z2 * z2);
        double den = (w2 + p1 * p1) * (w2 + p2 * p2);
        return K * Math.sqrt(num / den);
    }

    public double phase(double w) {
        // phi = atan(w / z1) + atan(w / z2) - atan(w / p1) - atan(w / p2)
        double phi = Math.atan2(w, z1) + Math.atan2(w, z2)
                   - Math.atan2(w, p1) - Math.atan2(w, p2);
        return phi; // radians
    }
}
