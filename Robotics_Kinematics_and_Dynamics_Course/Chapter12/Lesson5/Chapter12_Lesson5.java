public class TwoLinkValidation {

    static class Params {
        double m1, m2;
        double l1, c1, c2;
        double I1, I2;
        double g;
    }

    static double[][] M_lagrange_2R(double[] q, Params p) {
        double q2 = q[1];
        double a = p.I1 + p.I2 + p.m1 * p.c1 * p.c1
                   + p.m2 * (p.l1 * p.l1 + p.c2 * p.c2);
        double b = p.m2 * p.l1 * p.c2;
        double d = p.I2 + p.m2 * p.c2 * p.c2;
        double c2 = Math.cos(q2);

        double[][] M = new double[2][2];
        M[0][0] = a + 2.0 * b * c2;
        M[0][1] = d + b * c2;
        M[1][0] = M[0][1];
        M[1][1] = d;
        return M;
    }

    static double[] h_lagrange_2R(double[] q, double[] dq, Params p) {
        double q2  = q[1];
        double dq1 = dq[0];
        double dq2 = dq[1];
        double b   = p.m2 * p.l1 * p.c2;
        double s2  = Math.sin(q2);

        return new double[] {
            -b * s2 * (2.0 * dq1 * dq2 + dq2 * dq2),
             b * s2 * dq1 * dq1
        };
    }

    static double[] g_lagrange_2R(double[] q, Params p) {
        double q1 = q[0];
        double q2 = q[1];

        double g1 = p.g * (p.c1 * p.m1 * Math.cos(q1)
                + p.m2 * (p.l1 * Math.cos(q1)
                + p.c2 * Math.cos(q1 + q2)));
        double g2 = p.g * (p.m2 * p.c2 * Math.cos(q1 + q2));
        return new double[]{g1, g2};
    }

    static double[] tau_lagrange_2R(double[] q, double[] dq,
                                    double[] ddq, Params p) {
        double[][] M = M_lagrange_2R(q, p);
        double[] h   = h_lagrange_2R(q, dq, p);
        double[] g   = g_lagrange_2R(q, p);

        double[] tau = new double[2];
        for (int i = 0; i < 2; ++i) {
            tau[i] = h[i] + g[i];
            for (int j = 0; j < 2; ++j) {
                tau[i] += M[i][j] * ddq[j];
            }
        }
        return tau;
    }

    // --------------------------------------------------------------
    // Newton-Euler interface (to be provided by your Java robotics code).
    // gBase is a 3D vector [gx, gy, gz] in the base frame.
    // --------------------------------------------------------------
    static double[] tauNewtonEuler2R(double[] q, double[] dq,
                                     double[] ddq, Params p,
                                     double[] gBase) {
        throw new UnsupportedOperationException(
            "Connect to your Newton-Euler implementation."
        );
    }

    static double[][] reconstructM_NE(double[] q, Params p) {
        double[][] M = new double[2][2];
        double[] dq  = new double[]{0.0, 0.0};
        double[] gBase = new double[]{0.0, 0.0, 0.0};
        for (int j = 0; j < 2; ++j) {
            double[] ddq = new double[]{0.0, 0.0};
            ddq[j] = 1.0;
            double[] tau =
                tauNewtonEuler2R(q, dq, ddq, p, gBase);
            M[0][j] = tau[0];
            M[1][j] = tau[1];
        }
        return M;
    }

    static double[] reconstructG_NE(double[] q, Params p) {
        double[] dq   = new double[]{0.0, 0.0};
        double[] ddq  = new double[]{0.0, 0.0};
        double[] gBase = new double[]{0.0, -p.g, 0.0};
        return tauNewtonEuler2R(q, dq, ddq, p, gBase);
    }

    public static void main(String[] args) {
        Params p = new Params();
        p.m1 = 2.0; p.m2 = 1.0;
        p.l1 = 1.0; p.c1 = 0.5; p.c2 = 0.5;
        p.I1 = 0.2; p.I2 = 0.1;
        p.g  = 9.81;

        double[] q  = new double[]{0.3, -0.7};
        double[] dq = new double[]{0.1, -0.2};
        double[] ddq = new double[]{0.5, 0.3};

        double[] tauL =
            tau_lagrange_2R(q, dq, ddq, p);
        double[] tauNE =
            tauNewtonEuler2R(q, dq, ddq, p,
                             new double[]{0.0, -p.g, 0.0});

        System.out.printf("tau_L:  [%.6f, %.6f]%n",
                          tauL[0], tauL[1]);
        System.out.printf("tau_NE: [%.6f, %.6f]%n",
                          tauNE[0], tauNE[1]);
    }
}
      
