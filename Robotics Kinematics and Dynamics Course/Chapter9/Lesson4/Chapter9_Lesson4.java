public class Planar2RLoadCapacity {

    public static double[][] jacobian(double q1, double q2, double l1, double l2) {
        double s1  = Math.sin(q1);
        double c1  = Math.cos(q1);
        double s12 = Math.sin(q1 + q2);
        double c12 = Math.cos(q1 + q2);

        double[][] J = new double[2][2];
        J[0][0] = -l1 * s1 - l2 * s12;
        J[0][1] = -l2 * s12;
        J[1][0] =  l1 * c1 + l2 * c12;
        J[1][1] =  l2 * c12;
        return J;
    }

    public static double directionalCapacityBox(double[][] J,
                                                double[] fDir,
                                                double[] tauMax,
                                                double eps) {
        double norm = Math.sqrt(fDir[0] * fDir[0] + fDir[1] * fDir[1]);
        if (norm < eps) {
            throw new IllegalArgumentException("Direction vector must be nonzero.");
        }
        double[] fHat = new double[2];
        fHat[0] = fDir[0] / norm;
        fHat[1] = fDir[1] / norm;

        // v = J^T * f_hat
        double[] v = new double[2];
        v[0] = J[0][0] * fHat[0] + J[1][0] * fHat[1];
        v[1] = J[0][1] * fHat[0] + J[1][1] * fHat[1];

        double lambdaStar = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 2; ++i) {
            double v_i = v[i];
            if (Math.abs(v_i) < eps) {
                continue;
            }
            double lambda_i = tauMax[i] / Math.abs(v_i);
            if (lambda_i < lambdaStar) {
                lambdaStar = lambda_i;
            }
        }
        return lambdaStar;
    }

    public static void main(String[] args) {
        double l1 = 1.0, l2 = 1.0;
        double q1 = 0.0, q2 = 0.0;
        double[][] J = jacobian(q1, q2, l1, l2);
        double[] tauMax = {10.0, 10.0};
        double[] fDir = {0.0, -1.0};

        double lambdaStar = directionalCapacityBox(J, fDir, tauMax, 1e-9);
        System.out.println("Directional capacity lambda* = " + lambdaStar);
    }
}
      
