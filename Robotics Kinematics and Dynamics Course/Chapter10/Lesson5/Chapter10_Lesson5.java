public class Planar2RDynamics {

    public static class Params {
        public double m1, m2;
        public double l1, lc1, lc2;
        public double I1, I2;
        public double g0;
    }

    /**
     * Compute M(q), C(q, qdot) qdot, g(q) for a planar 2R arm.
     *
     * q      : length-2 array {q1, q2}
     * qdot   : length-2 array {dq1, dq2}
     * params : link parameters
     *
     * Returns an object with fields M (2x2), Cqdot (2), g (2).
     */
    public static class Result {
        public double[][] M = new double[2][2];
        public double[] Cqdot = new double[2];
        public double[] g = new double[2];
    }

    public static Result compute(double[] q, double[] qdot, Params p) {
        Result r = new Result();

        double q1 = q[0];
        double q2 = q[1];
        double dq1 = qdot[0];
        double dq2 = qdot[1];

        double c2 = Math.cos(q2);
        double s2 = Math.sin(q2);

        double m1  = p.m1;
        double m2  = p.m2;
        double l1  = p.l1;
        double lc1 = p.lc1;
        double lc2 = p.lc2;
        double I1  = p.I1;
        double I2  = p.I2;
        double g0  = p.g0;

        // Inertia matrix M(q)
        double M11 = I1 + I2 + m1 * lc1 * lc1
                     + m2 * (l1 * l1 + lc2 * lc2 + 2.0 * l1 * lc2 * c2);
        double M12 = I2 + m2 * (lc2 * lc2 + l1 * lc2 * c2);
        double M22 = I2 + m2 * lc2 * lc2;

        r.M[0][0] = M11;
        r.M[0][1] = M12;
        r.M[1][0] = M12;
        r.M[1][1] = M22;

        // Coriolis/centrifugal vector
        double h1 = -m2 * l1 * lc2 * s2 * (2.0 * dq1 * dq2 + dq2 * dq2);
        double h2 =  m2 * l1 * lc2 * s2 * dq1 * dq1;

        r.Cqdot[0] = h1;
        r.Cqdot[1] = h2;

        // Gravity vector
        double g1 = (m1 * lc1 + m2 * l1) * g0 * Math.cos(q1)
                    + m2 * lc2 * g0 * Math.cos(q1 + q2);
        double g2 = m2 * lc2 * g0 * Math.cos(q1 + q2);

        r.g[0] = g1;
        r.g[1] = g2;

        return r;
    }
}
      
