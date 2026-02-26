public class AxisAngleSO3 {

    public static double[][] skew(double[] w) {
        double wx = w[0], wy = w[1], wz = w[2];
        return new double[][]{
            { 0.0,  -wz,   wy},
            { wz,    0.0, -wx},
            {-wy,    wx,   0.0}
        };
    }

    public static double[][] axisAngleToR(double[] axis, double theta) {
        double[] w = axis.clone();
        double norm = Math.sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
        if (norm < 1e-12) {
            return new double[][]{
                {1.0, 0.0, 0.0},
                {0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0}
            };
        }
        w[0] /= norm; w[1] /= norm; w[2] /= norm;

        double[][] K = skew(w);
        double[][] K2 = matMul(K, K);
        double[][] R = new double[3][3];

        double s = Math.sin(theta);
        double c = Math.cos(theta);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double delta = (i == j) ? 1.0 : 0.0;
                R[i][j] = delta + s * K[i][j] + (1.0 - c) * K2[i][j];
            }
        }
        return R;
    }

    public static double[][] matMul(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    sum += A[i][k] * B[k][j];
                }
                C[i][j] = sum;
            }
        }
        return C;
    }

    public static AxisAngle RToAxisAngle(double[][] R) {
        double trace = R[0][0] + R[1][1] + R[2][2];
        double cosTheta = (trace - 1.0) / 2.0;
        cosTheta = Math.max(-1.0, Math.min(1.0, cosTheta));
        double theta = Math.acos(cosTheta);

        double eps = 1e-8;
        double[] axis = new double[3];
        if (theta < eps) {
            axis[0] = 1.0; axis[1] = 0.0; axis[2] = 0.0;
            theta = 0.0;
        } else {
            double denom = 2.0 * Math.sin(theta);
            axis[0] = (R[2][1] - R[1][2]) / denom;
            axis[1] = (R[0][2] - R[2][0]) / denom;
            axis[2] = (R[1][0] - R[0][1]) / denom;
            double n = Math.sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
            axis[0] /= n; axis[1] /= n; axis[2] /= n;
        }
        return new AxisAngle(axis, theta);
    }

    public static class AxisAngle {
        public final double[] axis;
        public final double theta;
        public AxisAngle(double[] axis, double theta) {
            this.axis = axis;
            this.theta = theta;
        }
    }
}
      
