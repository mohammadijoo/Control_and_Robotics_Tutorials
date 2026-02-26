class Link {
    double m;
    double[][] I;      // 3x3 inertia matrix
    double[] rCom;     // length-3 vector
    char jointType;    // 'R' or 'P'

    Link(double mass, double[][] inertia, double[] rCom, char jointType) {
        this.m = mass;
        this.I = inertia;
        this.rCom = rCom;
        this.jointType = jointType;
    }
}

public class NewtonEuler {

    private static final double[] Z_AXIS = new double[]{0.0, 0.0, 1.0};

    // Vector and matrix helpers (3D) omitted for brevity:
    // add(a,b), sub(a,b), cross(a,b), matVec(R,v), etc.
    // Assume they allocate small arrays and perform 3x3 ops.

    public static double[] inverseDynamics(
            double[] q,
            double[] qd,
            double[] qdd,
            double[] g,
            double[][][] R,
            double[][] p,
            Link[] links) {

        int n = links.length;

        double[][] w = new double[n + 1][3];
        double[][] wd = new double[n + 1][3];
        double[][] a = new double[n + 1][3];

        // a[0] = -g
        a[0][0] = -g[0];
        a[0][1] = -g[1];
        a[0][2] = -g[2];

        // forward recursion
        for (int i = 1; i <= n; ++i) {
            double[][] R_i = R[i - 1];
            double[] p_i = p[i - 1];
            Link link = links[i - 1];

            if (link.jointType == 'R') {
                double[] wPrev = w[i - 1];
                w[i] = add(matVec(R_i, wPrev), scale(Z_AXIS, qd[i - 1]));
                wd[i] = add(
                        add(matVec(R_i, wd[i - 1]),
                            scale(Z_AXIS, qdd[i - 1])),
                        cross(w[i], scale(Z_AXIS, qd[i - 1])));
                a[i] = matVec(R_i,
                        add(a[i - 1],
                            add(cross(wd[i - 1], p_i),
                                cross(w[i - 1], cross(w[i - 1], p_i)))));
            } else {
                // prismatic (simplified)
                w[i] = matVec(R_i, w[i - 1]);
                wd[i] = matVec(R_i, wd[i - 1]);
                a[i] = add(
                        matVec(R_i,
                               add(a[i - 1],
                                   add(cross(wd[i - 1], p_i),
                                       cross(w[i - 1], cross(w[i - 1], p_i))))),
                        add(scale(Z_AXIS, qdd[i - 1]),
                            scale(cross(w[i], Z_AXIS), 2.0 * qd[i - 1])));
            }
        }

        // forces and moments
        double[][] F = new double[n][3];
        double[][] N = new double[n][3];
        for (int i = 0; i < n; ++i) {
            Link link = links[i];
            double[] rC = link.rCom;
            double[] aC = add(a[i + 1],
                              add(cross(wd[i + 1], rC),
                                  cross(w[i + 1], cross(w[i + 1], rC))));
            F[i] = scale(aC, link.m);
            N[i] = add(matVec(link.I, wd[i + 1]),
                       cross(w[i + 1], matVec(link.I, w[i + 1])));
        }

        double[] fNext = new double[3];
        double[] nNext = new double[3];
        double[] tau = new double[n];

        for (int i = n - 1; i >= 0; --i) {
            double[][] R_ip1 = (i < n - 1) ? R[i] : identity3();
            double[] p_ip1 = (i < n - 1) ? p[i] : new double[3];
            double[] rC = links[i].rCom;

            double[] f_i = add(matVec(R_ip1, fNext), F[i]);
            double[] n_i = add(
                    add(N[i], matVec(R_ip1, nNext)),
                    add(cross(rC, F[i]),
                        cross(p_ip1, matVec(R_ip1, fNext))));

            if (links[i].jointType == 'R') {
                tau[i] = dot(n_i, Z_AXIS);
            } else {
                tau[i] = dot(f_i, Z_AXIS);
            }

            fNext = f_i;
            nNext = n_i;
        }
        return tau;
    }
}
      
