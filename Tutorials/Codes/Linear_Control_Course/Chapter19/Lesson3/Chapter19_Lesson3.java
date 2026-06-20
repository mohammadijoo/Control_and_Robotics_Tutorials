public class LeadDesigner {

    public static class LeadParams {
        public double alpha;
        public double tau;
        public double Kc;
        public LeadParams(double alpha, double tau, double Kc) {
            this.alpha = alpha;
            this.tau = tau;
            this.Kc = Kc;
        }
    }

    public static LeadParams designLead(double phiMaxDeg,
                                        double wcDes,
                                        double magC0G_at_wc) {
        double phiMax = Math.toRadians(phiMaxDeg);
        double s = Math.sin(phiMax);
        double alpha = (1.0 - s) / (1.0 + s);
        double tau = 1.0 / (wcDes * Math.sqrt(alpha));
        double Kc = 1.0 / magC0G_at_wc;
        return new LeadParams(alpha, tau, Kc);
    }

    public static void main(String[] args) {
        double phiMaxDeg = 45.0;
        double wcDes = 4.0;
        double magC0G_at_wc = 0.146; // from offline analysis

        LeadParams p = designLead(phiMaxDeg, wcDes, magC0G_at_wc);
        System.out.println("alpha = " + p.alpha);
        System.out.println("tau   = " + p.tau);
        System.out.println("Kc    = " + p.Kc);

        // The resulting C(s) can be discretized and implemented
        // in a Java-based control loop or simulation environment.
    }
}
