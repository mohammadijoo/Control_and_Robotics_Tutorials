public class SO3Demo {

    public static double[][] rotZ(double theta) {
        double c = Math.cos(theta);
        double s = Math.sin(theta);
        return new double[][] {
            { c, -s, 0.0 },
            { s,  c, 0.0 },
            { 0.0, 0.0, 1.0 }
        };
    }

    public static double[][] rotY(double theta) {
        double c = Math.cos(theta);
        double s = Math.sin(theta);
        return new double[][] {
            {  c, 0.0,  s },
            { 0.0, 1.0, 0.0 },
            { -s, 0.0,  c }
        };
    }

    public static double[][] matMul(double[][] A, double[][] B) {
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

    public static double[][] transpose(double[][] A) {
        double[][] T = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                T[i][j] = A[j][i];
            }
        }
        return T;
    }

    public static double frobNorm(double[][] A) {
        double s = 0.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                s += A[i][j] * A[i][j];
            }
        }
        return Math.sqrt(s);
    }

    public static double det3(double[][] R) {
        return
            R[0][0]*(R[1][1]*R[2][2] - R[1][2]*R[2][1]) -
            R[0][1]*(R[1][0]*R[2][2] - R[1][2]*R[2][0]) +
            R[0][2]*(R[1][0]*R[2][1] - R[1][1]*R[2][0]);
    }

    public static boolean isSO3(double[][] R, double tol) {
        double[][] I = {
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0}
        };
        double[][] RT = transpose(R);
        double[][] RT_R = matMul(RT, R);
        double[][] diff = new double[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                diff[i][j] = RT_R[i][j] - I[i][j];
            }
        }
        double orthErr = frobNorm(diff);
        double detR = det3(R);
        return orthErr < tol && Math.abs(detR - 1.0) < tol;
    }

    public static void main(String[] args) {
        double theta = Math.toRadians(30.0);
        double[][] Rz = rotZ(theta);
        double[][] Ry = rotY(theta);
        double[][] R = matMul(Rz, Ry);

        System.out.println("det(R) = " + det3(R));
        System.out.println("R in SO(3)? " + isSO3(R, 1e-9));
    }
}
      
