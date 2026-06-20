public class Link {
    public double[][] R;   // 3x3 rotation matrix
    public double[] p;     // length-3 vector
    public double[] rc;    // length-3 vector
    public char jointType; // 'R' or 'P'
}

public class ForwardRecursionResult {
    public double[][] omega; // n x 3
    public double[][] alpha; // n x 3
    public double[][] a;     // n x 3
    public double[][] ac;    // n x 3
}

public class NewtonEulerForward {

    private static double[] vec(double x, double y, double z) {
        return new double[]{x, y, z};
    }

    private static double[] add(double[] a, double[] b) {
        return new double[]{a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }

    private static double[] sub(double[] a, double[] b) {
        return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double[] scale(double[] a, double s) {
        return new double[]{s * a[0], s * a[1], s * a[2]};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        };
    }

    private static double[] matVec(double[][] M, double[] v) {
        return new double[]{
            M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
            M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
            M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]
        };
    }

    public static ForwardRecursionResult forwardRecursion(
            Link[] links, double[] q, double[] qd, double[] qdd,
            double[] g) {

        int n = links.length;
        double[] z = new double[]{0.0, 0.0, 1.0};

        ForwardRecursionResult res = new ForwardRecursionResult();
        res.omega = new double[n][3];
        res.alpha = new double[n][3];
        res.a = new double[n][3];
        res.ac = new double[n][3];

        double[] omegaPrev = vec(0.0, 0.0, 0.0);
        double[] alphaPrev = vec(0.0, 0.0, 0.0);
        double[] aPrev = scale(g, -1.0); // base acceleration

        for (int i = 0; i < n; ++i) {
            Link L = links[i];
            double qi = q[i];
            double qdi = qd[i];
            double qddi = qdd[i];

            double[] aBase = add(
                add(aPrev, cross(alphaPrev, L.p)),
                cross(omegaPrev, cross(omegaPrev, L.p))
            );

            double[] omega_i, alpha_i, a_i;

            if (L.jointType == 'R') {
                omega_i = add(matVec(L.R, omegaPrev), scale(z, qdi));
                alpha_i = add(
                    matVec(L.R, alphaPrev),
                    add(scale(z, qddi), cross(omega_i, scale(z, qdi)))
                );
                a_i = matVec(L.R, aBase);
            } else if (L.jointType == 'P') {
                omega_i = matVec(L.R, omegaPrev);
                alpha_i = matVec(L.R, alphaPrev);
                a_i = add(
                    matVec(L.R, aBase),
                    add(scale(z, qddi), cross(scale(2.0, omega_i), scale(z, qdi)))
                );
            } else {
                throw new IllegalArgumentException("jointType must be 'R' or 'P'");
            }

            double[] ac_i = add(
                add(a_i, cross(alpha_i, L.rc)),
                cross(omega_i, cross(omega_i, L.rc))
            );

            res.omega[i] = omega_i;
            res.alpha[i] = alpha_i;
            res.a[i] = a_i;
            res.ac[i] = ac_i;

            omegaPrev = omega_i;
            alphaPrev = alpha_i;
            aPrev = a_i;
        }

        return res;
    }
}
      
