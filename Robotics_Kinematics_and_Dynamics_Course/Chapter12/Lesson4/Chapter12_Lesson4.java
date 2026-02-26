public class NewtonEuler {

    public static class Link {
        public double[][] R;  // 3x3
        public double[] p;    // length 3
        public double[] r_c;  // length 3
        public double m;
        public double[][] I;  // 3x3
        public char jointType; // 'R' or 'P'
    }

    private static final double[] Z_AXIS = new double[]{0.0, 0.0, 1.0};

    // --- vector and matrix helpers omitted for brevity (add, cross, dot, matVec, etc.)

    public static double[] newtonEuler(
            Link[] links,
            double[] q, double[] qd, double[] qdd,
            double[] g0
    ) {
        int n = links.length;
        double[][] omega = new double[n+1][3];
        double[][] omegadot = new double[n+1][3];
        double[][] a = new double[n+1][3];
        double[][] a_c = new double[n][3];

        // base
        a[0] = vecScale(g0, -1.0);

        // forward recursion
        for (int i = 1; i <= n; ++i) {
            Link L = links[i-1];
            if (L.jointType == 'R') {
                omega[i] = vecAdd(
                        matVec(L.R, omega[i-1]),
                        vecScale(Z_AXIS, qd[i-1])
                );
                omegadot[i] = vecAdd(
                        matVec(L.R, omegadot[i-1]),
                        vecAdd(
                            vecScale(Z_AXIS, qdd[i-1]),
                            cross(omega[i], vecScale(Z_AXIS, qd[i-1]))
                        )
                );
                double[] term = vecAdd(
                        omegadot[i-1],
                        cross(omega[i-1], cross(omega[i-1], L.p))
                );
                a[i] = matVec(L.R, vecAdd(a[i-1], term));
            } else {
                omega[i] = matVec(L.R, omega[i-1]);
                omegadot[i] = matVec(L.R, omegadot[i-1]);
                double[] term = vecAdd(
                        omegadot[i-1],
                        cross(omega[i-1], cross(omega[i-1], L.p))
                );
                double[] baseA = matVec(L.R, vecAdd(a[i-1], term));
                a[i] = vecAdd(
                        vecAdd(
                            baseA,
                            vecScale(cross(omega[i], vecScale(Z_AXIS, qd[i-1])), 2.0)
                        ),
                        vecScale(Z_AXIS, qdd[i-1])
                );
            }
            a_c[i-1] = vecAdd(
                    vecAdd(a[i], cross(omegadot[i], L.r_c)),
                    cross(omega[i], cross(omega[i], L.r_c))
            );
        }

        // backward recursion
        double[][] f = new double[n+2][3];
        double[][] nvec = new double[n+2][3];
        double[] tau = new double[n];

        for (int i = n; i >= 1; --i) {
            Link L = links[i-1];

            double[][] R_next = identity3();
            double[] p_next = new double[3];
            if (i < n) {
                R_next = transpose(links[i].R);
                p_next = links[i].p;
            }

            f[i] = vecAdd(
                    vecScale(a_c[i-1], L.m),
                    matVec(R_next, f[i+1])
            );
            double[] Iomega = matVec(L.I, omega[i]);
            nvec[i] = vecAdd(
                    vecAdd(
                        vecAdd(
                            matVec(L.I, omegadot[i]),
                            cross(omega[i], Iomega)
                        ),
                        cross(L.r_c, vecScale(a_c[i-1], L.m))
                    ),
                    vecAdd(
                        matVec(R_next, nvec[i+1]),
                        cross(vecAdd(p_next, L.r_c), matVec(R_next, f[i+1]))
                    )
            );

            if (L.jointType == 'R') {
                tau[i-1] = dot(nvec[i], Z_AXIS);
            } else {
                tau[i-1] = dot(f[i], Z_AXIS);
            }
        }
        return tau;
    }
}
      
