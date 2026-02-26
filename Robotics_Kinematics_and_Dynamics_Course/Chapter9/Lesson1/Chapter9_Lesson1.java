public class WrenchTransform {

    // Compute p x f
    public static double[] cross(double[] p, double[] f) {
        return new double[] {
            p[1] * f[2] - p[2] * f[1],
            p[2] * f[0] - p[0] * f[2],
            p[0] * f[1] - p[1] * f[0]
        };
    }

    // Multiply 3x3 matrix R by 3x1 vector v
    public static double[] matVec(double[][] R, double[] v) {
        return new double[] {
            R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
            R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
            R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
        };
    }

    public static double[] transformWrench(double[][] R, double[] p, double[] wB) {
        // wB = [fB(0:2); mB(3:5)]
        double[] fB = new double[] { wB[0], wB[1], wB[2] };
        double[] mB = new double[] { wB[3], wB[4], wB[5] };

        double[] fA = matVec(R, fB);
        double[] px_fA = cross(p, fA);
        double[] mA_rot = matVec(R, mB);

        double[] mA = new double[] {
            px_fA[0] + mA_rot[0],
            px_fA[1] + mA_rot[1],
            px_fA[2] + mA_rot[2]
        };

        return new double[] { fA[0], fA[1], fA[2], mA[0], mA[1], mA[2] };
    }

    public static void main(String[] args) {
        double[][] R = {
            {0.0, 0.0, 1.0},
            {0.0, 1.0, 0.0},
            {-1.0, 0.0, 0.0}
        };
        double[] p = {0.5, 0.0, 0.0};
        double[] wB = {0.0, 0.0, 10.0, 0.0, 0.0, 0.0};

        double[] wA = transformWrench(R, p, wB);
        System.out.println("wA = ");
        for (double x : wA) {
            System.out.println(x);
        }
    }
}
      
