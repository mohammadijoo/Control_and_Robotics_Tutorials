
public class GravityComp2Link {

    private final double m1, m2;
    private final double l1, l2;
    private final double g0;

    public GravityComp2Link(double m1, double m2,
                            double l1, double l2,
                            double g0) {
        this.m1 = m1;
        this.m2 = m2;
        this.l1 = l1;
        this.l2 = l2;
        this.g0 = g0;
    }

    public double[] gravityTorque(double q1, double q2) {
        double c1 = Math.cos(q1);
        double c12 = Math.cos(q1 + q2);

        double g1 = (m1 * l1 / 2.0 + m2 * l1) * g0 * c1
                  +  m2 * l2 / 2.0 * g0 * c12;
        double g2 =  m2 * l2 / 2.0 * g0 * c12;

        return new double[]{g1, g2};
    }

    public double[] pdGravityControl(double q1, double q2,
                                     double dq1, double dq2,
                                     double q1_d, double q2_d,
                                     double dq1_d, double dq2_d,
                                     double Kp1, double Kp2,
                                     double Kd1, double Kd2) {

        double e1  = q1_d  - q1;
        double e2  = q2_d  - q2;
        double de1 = dq1_d - dq1;
        double de2 = dq2_d - dq2;

        double tau_pd1 = Kp1 * e1 + Kd1 * de1;
        double tau_pd2 = Kp2 * e2 + Kd2 * de2;

        double[] g = gravityTorque(q1, q2);
        double tau1 = tau_pd1 + g[0];
        double tau2 = tau_pd2 + g[1];

        return new double[]{tau1, tau2};
    }
}
