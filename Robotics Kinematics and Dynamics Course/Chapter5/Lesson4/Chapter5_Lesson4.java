public class RedundantFK {

    public static double[][] matMul(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                double sum = 0.0;
                for (int k = 0; k < 4; k++) {
                    sum += A[i][k] * B[k][j];
                }
                C[i][j] = sum;
            }
        }
        return C;
    }

    public static double[][] dhTransform(double a, double alpha,
                                         double d, double theta) {
        double ca = Math.cos(alpha);
        double sa = Math.sin(alpha);
        double ct = Math.cos(theta);
        double st = Math.sin(theta);

        double[][] T = new double[4][4];
        T[0][0] = ct;   T[0][1] = -st * ca;  T[0][2] =  st * sa;  T[0][3] = a * ct;
        T[1][0] = st;   T[1][1] =  ct * ca;  T[1][2] = -ct * sa;  T[1][3] = a * st;
        T[2][0] = 0.0;  T[2][1] =  sa;       T[2][2] =  ca;       T[2][3] = d;
        T[3][0] = 0.0;  T[3][1] = 0.0;       T[3][2] = 0.0;       T[3][3] = 1.0;
        return T;
    }

    public static double[][] fkDH(double[][] dh, double[] q) {
        int n = dh.length;
        if (q.length != n) {
            throw new IllegalArgumentException("fkDH: size mismatch.");
        }
        double[][] T = new double[4][4];
        for (int i = 0; i < 4; i++) {
            T[i][i] = 1.0;
        }
        for (int i = 0; i < n; i++) {
            double a = dh[i][0];
            double alpha = dh[i][1];
            double d = dh[i][2];
            double thetaOffset = dh[i][3];
            boolean revolute = (dh[i][4] > 0.5); // 1.0 for revolute, 0.0 for prismatic

            double theta = thetaOffset;
            double di = d;
            if (revolute) {
                theta += q[i];
            } else {
                di += q[i];
            }
            double[][] Ti = dhTransform(a, alpha, di, theta);
            T = matMul(T, Ti);
        }
        return T;
    }

    public static void main(String[] args) {
        // DH table for a simple 7-DOF redundant arm (toy values)
        double[][] dh = new double[7][5];
        for (int i = 0; i < 7; i++) {
            dh[i][0] = 0.2;   // a
            dh[i][1] = 0.0;   // alpha
            dh[i][2] = 0.0;   // d
            dh[i][3] = 0.0;   // theta offset
            dh[i][4] = 1.0;   // revolute flag
        }

        double[] q = new double[] {0.0, 0.3, -0.5, 0.4, -0.2, 0.1, 0.05};
        double[][] T = fkDH(dh, q);

        System.out.println("End-effector transform:");
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                System.out.printf("%8.4f ", T[i][j]);
            }
            System.out.println();
        }
    }
}
      
