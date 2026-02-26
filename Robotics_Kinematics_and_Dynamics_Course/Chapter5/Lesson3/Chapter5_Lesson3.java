public class DHKinematics {

    public static double[][] dhTransform(double theta, double d,
                                         double a, double alpha) {
        double ct = Math.cos(theta);
        double st = Math.sin(theta);
        double ca = Math.cos(alpha);
        double sa = Math.sin(alpha);

        double[][] A = new double[4][4];
        A[0][0] = ct;          A[0][1] = -st * ca;  A[0][2] =  st * sa;  A[0][3] = a * ct;
        A[1][0] = st;          A[1][1] =  ct * ca;  A[1][2] = -ct * sa;  A[1][3] = a * st;
        A[2][0] = 0.0;         A[2][1] =      sa;  A[2][2] =      ca;  A[2][3] = d;
        A[3][0] = 0.0;         A[3][1] =     0.0;  A[3][2] =     0.0;  A[3][3] = 1.0;
        return A;
    }

    public static double[][] matMul4(double[][] X, double[][] Y) {
        double[][] Z = new double[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k) {
                    s += X[i][k] * Y[k][j];
                }
                Z[i][j] = s;
            }
        }
        return Z;
    }

    public static double[][] fkDH(double[] q,
                                  double[] a,
                                  double[] alpha,
                                  double[] d,
                                  double[] thetaOffset) {
        int n = q.length;
        double[][] T = new double[4][4];
        // initialize T as identity
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                T[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
        for (int i = 0; i < n; ++i) {
            double theta_i = q[i] + thetaOffset[i];
            double[][] A_i = dhTransform(theta_i, d[i], a[i], alpha[i]);
            T = matMul4(T, A_i);
        }
        return T;
    }

    // Planar 2R example
    public static double[][] fkPlanar2R(double q1, double q2,
                                        double l1, double l2) {
        double[] q = new double[] { q1, q2 };
        double[] a = new double[] { l1, l2 };
        double[] alpha = new double[] { 0.0, 0.0 };
        double[] d = new double[] { 0.0, 0.0 };
        double[] offs = new double[] { 0.0, 0.0 };
        return fkDH(q, a, alpha, d, offs);
    }
}
      
