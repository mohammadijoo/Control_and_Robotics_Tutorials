public class TwoLinkDynamics {

    public static class Params {
        public double l1, l2;
        public double c1, c2;
        public double m1, m2;
        public double I1, I2;
        public double g;

        public Params(double l1, double l2,
                      double c1, double c2,
                      double m1, double m2,
                      double I1, double I2,
                      double g) {
            this.l1 = l1;
            this.l2 = l2;
            this.c1 = c1;
            this.c2 = c2;
            this.m1 = m1;
            this.m2 = m2;
            this.I1 = I1;
            this.I2 = I2;
            this.g = g;
        }
    }

    public static double[][] M(double[] q, Params p) {
        double q2 = q[1];
        double cos2 = Math.cos(q2);

        double M11 = p.I1 + p.I2
                     + p.m1 * p.c1 * p.c1
                     + p.m2 * (p.l1 * p.l1 + p.c2 * p.c2 + 2.0 * p.l1 * p.c2 * cos2);
        double M12 = p.I2 + p.m2 * (p.c2 * p.c2 + p.l1 * p.c2 * cos2);
        double M22 = p.I2 + p.m2 * p.c2 * p.c2;

        return new double[][] {
            { M11, M12 },
            { M12, M22 }
        };
    }

    public static double[][] C(double[] q, double[] qdot, Params p) {
        double q2 = q[1];
        double q1dot = qdot[0];
        double q2dot = qdot[1];

        double sin2 = Math.sin(q2);
        double h = p.m2 * p.l1 * p.c2 * sin2;

        return new double[][] {
            { -h * q2dot,      -h * (q1dot + q2dot) },
            {  h * q1dot,       0.0 }
        };
    }

    public static double[] g(double[] q, Params p) {
        double q1 = q[0];
        double q2 = q[1];

        double g1 = (p.m1 * p.c1 + p.m2 * p.l1) * p.g * Math.cos(q1)
                    + p.m2 * p.c2 * p.g * Math.cos(q1 + q2);
        double g2 = p.m2 * p.c2 * p.g * Math.cos(q1 + q2);

        return new double[] { g1, g2 };
    }

    public static double[] forwardDynamics(double[] q,
                                           double[] qdot,
                                           double[] tau,
                                           Params p) {
        double[][] M = M(q, p);
        double[][] C = C(q, qdot, p);
        double[] g = g(q, p);

        // rhs = tau - C qdot - g
        double[] rhs = new double[2];
        rhs[0] = tau[0]
                 - (C[0][0] * qdot[0] + C[0][1] * qdot[1])
                 - g[0];
        rhs[1] = tau[1]
                 - (C[1][0] * qdot[0] + C[1][1] * qdot[1])
                 - g[1];

        // Invert 2x2 inertia matrix
        double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        double inv00 =  M[1][1] / det;
        double inv01 = -M[0][1] / det;
        double inv10 = -M[1][0] / det;
        double inv11 =  M[0][0] / det;

        double[] qddot = new double[2];
        qddot[0] = inv00 * rhs[0] + inv01 * rhs[1];
        qddot[1] = inv10 * rhs[0] + inv11 * rhs[1];
        return qddot;
    }
}
      
