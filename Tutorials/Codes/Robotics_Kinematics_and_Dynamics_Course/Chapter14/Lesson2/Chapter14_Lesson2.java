public class GearedJoint {
    private final double Jm, Jg, JL, bm, bq, n, eta;
    private final double J_eq, b_eq;

    public GearedJoint(double Jm, double Jg, double JL,
                       double bm, double bq, double n, double eta) {
        this.Jm = Jm;
        this.Jg = Jg;
        this.JL = JL;
        this.bm = bm;
        this.bq = bq;
        this.n  = n;
        this.eta = eta;

        this.J_eq = n * n * (Jm + Jg) + JL;
        this.b_eq = n * n * bm + bq;
    }

    public double jointAcceleration(double q, double qdot,
                                    double tau_m, double tau_ext) {
        double rhs = n * eta * tau_m + tau_ext - b_eq * qdot;
        return rhs / J_eq;
    }

    public double getEquivalentInertia() {
        return J_eq;
    }

    public static void main(String[] args) {
        GearedJoint joint = new GearedJoint(
            0.002, 0.001, 0.05,   // Jm, Jg, JL
            0.001, 0.02,          // bm, bq
            80.0, 0.9             // n, eta
        );

        double q = 0.0;
        double qdot = 0.0;
        double dt = 0.001;

        for (int k = 0; k < 5000; ++k) {
            double t = k * dt;
            double tau_m = (t > 0.05) ? 1.0 : 0.0; // delayed step
            double qdd = joint.jointAcceleration(q, qdot, tau_m, 0.0);
            qdot += qdd * dt;
            q += qdot * dt;
        }

        System.out.println("Equivalent inertia J_eq = " + joint.getEquivalentInertia());
        System.out.println("Final joint angle q = " + q + " rad");
    }
}
      
