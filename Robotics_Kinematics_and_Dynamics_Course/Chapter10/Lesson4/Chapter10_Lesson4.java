public class Planar2R {
    public double l1, l2, lc1, lc2;
    public double m1, m2, I1, I2, g;

    public Planar2R(double l1, double l2, double lc1, double lc2,
                    double m1, double m2, double I1, double I2, double g) {
        this.l1 = l1; this.l2 = l2;
        this.lc1 = lc1; this.lc2 = lc2;
        this.m1 = m1; this.m2 = m2;
        this.I1 = I1; this.I2 = I2;
        this.g = g;
    }

    public double[][] M(double[] q) {
        double q1 = q[0];
        double q2 = q[1];
        double c2 = Math.cos(q2);

        double m11 = I1 + I2
                     + m1 * lc1 * lc1
                     + m2 * (l1 * l1 + lc2 * lc2 + 2.0 * l1 * lc2 * c2);
        double m12 = I2 + m2 * (lc2 * lc2 + l1 * lc2 * c2);
        double m22 = I2 + m2 * lc2 * lc2;

        return new double[][] {
            {m11, m12},
            {m12, m22}
        };
    }

    public double[][] C(double[] q, double[] qd) {
        double q1 = q[0];
        double q2 = q[1];
        double q1d = qd[0];
        double q2d = qd[1];
        double s2 = Math.sin(q2);
        double h = -m2 * l1 * lc2 * s2;

        double c11 = h * q2d;
        double c12 = h * (q1d + q2d);
        double c21 = -h * q1d;
        double c22 = 0.0;

        return new double[][] {
            {c11, c12},
            {c21, c22}
        };
    }

    public double[] gVec(double[] q) {
        double q1 = q[0];
        double q2 = q[1];

        double g1 = ((m1 * lc1 + m2 * l1) * g * Math.cos(q1)
                     + m2 * lc2 * g * Math.cos(q1 + q2));
        double g2 = m2 * lc2 * g * Math.cos(q1 + q2);

        return new double[] {g1, g2};
    }

    public double[] dynamics(double[] q, double[] qd, double[] u) {
        double[][] Mq = M(q);
        double[][] Cq = C(q, qd);
        double[] gq = gVec(q);

        double[] Cqd = new double[2];
        Cqd[0] = Cq[0][0] * qd[0] + Cq[0][1] * qd[1];
        Cqd[1] = Cq[1][0] * qd[0] + Cq[1][1] * qd[1];

        double[] rhs = new double[2];
        rhs[0] = u[0] - Cqd[0] - gq[0];
        rhs[1] = u[1] - Cqd[1] - gq[1];

        // Solve 2x2 linear system Mq * qdd = rhs analytically
        double det = Mq[0][0] * Mq[1][1] - Mq[0][1] * Mq[1][0];
        double inv00 =  Mq[1][1] / det;
        double inv01 = -Mq[0][1] / det;
        double inv10 = -Mq[1][0] / det;
        double inv11 =  Mq[0][0] / det;

        double[] qdd = new double[2];
        qdd[0] = inv00 * rhs[0] + inv01 * rhs[1];
        qdd[1] = inv10 * rhs[0] + inv11 * rhs[1];
        return qdd;
    }
}
      
