public class PendulumLinearization {

    public static double[][] computeA(double m, double L, double b, double g,
                                      double thetaOp, double omegaOp) {

        // omegaOp is not needed explicitly here but included for completeness
        double[][] A = new double[2][2];
        A[0][0] = 0.0;
        A[0][1] = 1.0;

        A[1][0] = -g / L * Math.cos(thetaOp);
        A[1][1] = -b / (m * L * L);
        return A;
    }

    public static double[] computeB(double m, double L) {
        double[] B = new double[2];
        B[0] = 0.0;
        B[1] = 1.0 / (m * L * L);
        return B;
    }

    public static void main(String[] args) {
        double m = 1.0, L = 1.0, b = 0.1, g = 9.81;
        double thetaOp = 0.0, omegaOp = 0.0;

        double[][] A = computeA(m, L, b, g, thetaOp, omegaOp);
        double[] B = computeB(m, L);

        System.out.println("A matrix:");
        for (int i = 0; i < 2; ++i) {
            System.out.println(A[i][0] + "  " + A[i][1]);
        }

        System.out.println("\nB vector:");
        System.out.println(B[0] + "  " + B[1]);
    }
}
