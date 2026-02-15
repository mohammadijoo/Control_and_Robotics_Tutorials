public class VelocityPropagation {

    // Multiply 4x4 matrices: C = A * B
    public static double[][] mul4(double[][] A, double[][] B) {
        double[][] C = new double[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 4; ++k) {
                    sum += A[i][k] * B[k][j];
                }
                C[i][j] = sum;
            }
        }
        return C;
    }

    // Compute 3x3 skew matrix of w
    public static double[][] skew(double[] w) {
        double wx = w[0], wy = w[1], wz = w[2];
        return new double[][] {
            { 0.0, -wz,  wy },
            { wz,  0.0, -wx },
            { -wy, wx,  0.0 }
        };
    }

    // Compute 6x6 adjoint of T (4x4)
    public static double[][] adjoint(double[][] T) {
        double[][] R = new double[3][3];
        double[] p = new double[3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R[i][j] = T[i][j];
            }
            p[i] = T[i][3];
        }
        double[][] pHat = skew(p);

        double[][] Ad = new double[6][6];
        // Top-left (R)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Ad[i][j] = R[i][j];
            }
        }
        // Bottom-left (pHat * R)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    sum += pHat[i][k] * R[k][j];
                }
                Ad[3 + i][j] = sum;
            }
        }
        // Bottom-right (R)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Ad[3 + i][3 + j] = R[i][j];
            }
        }
        return Ad;
    }

    // Multiply 6x6 matrix by 6x1 vector
    public static double[] mul6(double[][] A, double[] x) {
        double[] y = new double[6];
        for (int i = 0; i < 6; ++i) {
            double sum = 0.0;
            for (int j = 0; j < 6; ++j) {
                sum += A[i][j] * x[j];
            }
            y[i] = sum;
        }
        return y;
    }

    // Add two 6x1 vectors
    public static double[] add6(double[] a, double[] b) {
        double[] c = new double[6];
        for (int i = 0; i < 6; ++i) {
            c[i] = a[i] + b[i];
        }
        return c;
    }

    // Scale 6x1 vector
    public static double[] scale6(double[] a, double s) {
        double[] c = new double[6];
        for (int i = 0; i < 6; ++i) {
            c[i] = s * a[i];
        }
        return c;
    }

    // Forward velocity propagation
    public static double[][] forwardVelocityChain(
            double[][][] Tlist, // [n][4][4]
            double[][] Slist,   // [n][6]
            double[] qdot,      // [n]
            double[] V0         // [6], may be null
    ) {
        int n = Slist.length;
        double[] Vprev = new double[6];
        if (V0 != null) {
            System.arraycopy(V0, 0, Vprev, 0, 6);
        }

        double[][] Vlinks = new double[n][6];

        for (int i = 0; i < n; ++i) {
            double[][] T_i = Tlist[i];
            // For brevity, assume T_i is already the inverse transform T_{i-1,i}^{-1}
            double[][] Ad_inv = adjoint(T_i);
            double[] VprevInI = mul6(Ad_inv, Vprev);
            double[] Si = Slist[i];
            double[] jointTwist = scale6(Si, qdot[i]);
            double[] Vi = add6(VprevInI, jointTwist);
            Vlinks[i] = Vi;
            Vprev = Vi;
        }
        return Vlinks;
    }
}
      
