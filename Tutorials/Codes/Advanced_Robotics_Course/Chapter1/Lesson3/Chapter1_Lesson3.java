public class Planar2RProjection {

    static double L1 = 1.0;
    static double L2 = 1.0;

    public static double[] fk(double[] q) {
        double theta1 = q[0];
        double theta2 = q[1];
        double x = L1 * Math.cos(theta1) + L2 * Math.cos(theta1 + theta2);
        double y = L1 * Math.sin(theta1) + L2 * Math.sin(theta1 + theta2);
        return new double[]{x, y};
    }

    public static double[][] jacobian(double[] q) {
        double theta1 = q[0];
        double theta2 = q[1];
        double s1  = Math.sin(theta1);
        double c1  = Math.cos(theta1);
        double s12 = Math.sin(theta1 + theta2);
        double c12 = Math.cos(theta1 + theta2);

        double[][] J = new double[2][2];
        J[0][0] = -L1 * s1 - L2 * s12;
        J[0][1] = -L2 * s12;
        J[1][0] =  L1 * c1 + L2 * c12;
        J[1][1] =  L2 * c12;
        return J;
    }

    private static double[][] inv2x2(double[][] A) {
        double a = A[0][0];
        double b = A[0][1];
        double c = A[1][0];
        double d = A[1][1];
        double det = a * d - b * c;
        double invDet = 1.0 / det;
        return new double[][]{
            {  d * invDet, -b * invDet},
            { -c * invDet,  a * invDet}
        };
    }

    public static double[] project(double[] qInit, double[] pDes,
                                   int maxIters, double tol) {
        double[] q = qInit.clone();
        for (int k = 0; k < maxIters; ++k) {
            double[] h = fk(q);
            h[0] -= pDes[0];
            h[1] -= pDes[1];

            double[][] J = jacobian(q);

            // J_pinv = J^T (J J^T)^{-1} for full-rank 2x2 J
            double[][] JJt = new double[2][2];
            // JJt = J * J^T
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    JJt[i][j] = J[i][0] * J[j][0] + J[i][1] * J[j][1];
                }
            }
            double[][] JJtInv = inv2x2(JJt);

            double[][] Jt = new double[2][2];
            Jt[0][0] = J[0][0]; Jt[0][1] = J[1][0];
            Jt[1][0] = J[0][1]; Jt[1][1] = J[1][1];

            double[][] Jpinv = new double[2][2];
            // Jpinv = J^T * JJtInv
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    Jpinv[i][j] = Jt[i][0] * JJtInv[0][j] + Jt[i][1] * JJtInv[1][j];
                }
            }

            // q = q - J_pinv * h
            double dq0 = Jpinv[0][0] * h[0] + Jpinv[0][1] * h[1];
            double dq1 = Jpinv[1][0] * h[0] + Jpinv[1][1] * h[1];
            q[0] -= dq0;
            q[1] -= dq1;

            double normH = Math.sqrt(h[0] * h[0] + h[1] * h[1]);
            if (normH < tol) {
                break;
            }
        }
        return q;
    }
}
      
