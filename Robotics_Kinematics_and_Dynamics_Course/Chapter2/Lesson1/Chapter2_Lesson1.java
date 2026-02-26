public class RigidBodyConfig {

    // R is 3x3 row-major matrix, p is length-3 vector
    public double[][] R;
    public double[] p;

    public RigidBodyConfig(double[][] R, double[] p) {
        this.R = R;
        this.p = p;
    }

    public static double[][] identity3x3() {
        return new double[][] {
                {1.0, 0.0, 0.0},
                {0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0}
        };
    }

    public static double[][] transpose3x3(double[][] A) {
        double[][] T = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                T[i][j] = A[j][i];
            }
        }
        return T;
    }

    public static double[][] multiply3x3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                C[i][j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    public static double frobeniusNormDiff(double[][] A, double[][] B) {
        double s = 0.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                double d = A[i][j] - B[i][j];
                s += d * d;
            }
        }
        return Math.sqrt(s);
    }

    public static boolean isValidRotation(double[][] R, double tol) {
        double[][] RT = transpose3x3(R);
        double[][] RT_R = multiply3x3(RT, R);
        double[][] I = identity3x3();
        double err = frobeniusNormDiff(RT_R, I);
        // A full implementation would also check det(R) close to 1
        return err <= tol;
    }

    public static void main(String[] args) {
        double[][] R = identity3x3();
        double[] p = new double[] {0.0, 0.0, 0.0};
        RigidBodyConfig q = new RigidBodyConfig(R, p);
        System.out.println("Valid rotation: " + isValidRotation(q.R, 1e-6));

        // Joint-space example: planar 2R arm configuration
        double[] qJoint = new double[] {0.5, -0.3};
        System.out.println("q = [" + qJoint[0] + ", " + qJoint[1] + "]");
    }
}
      
