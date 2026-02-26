
public class PendulumLinearization {

    // Nonlinear dynamics: xdot = f(x,u)
    // x is length-2 array [q, qdot]
    public static double[] f(double[] x, double u,
                             double m, double g, double l, double I) {
        double[] xdot = new double[2];
        double q    = x[0];
        double qdot = x[1];

        xdot[0] = qdot;
        xdot[1] = -(m*g*l/I) * Math.sin(q) + (1.0/I) * u;
        return xdot;
    }

    // Finite-difference linearization
    public static void linearize(double[] x0, double u0,
                                 double m, double g, double l, double I,
                                 double eps,
                                 double[][] A, double[] B) {

        double[] f0 = f(x0, u0, m, g, l, I);

        // Columns of A
        for (int i = 0; i < 2; ++i) {
            double[] xPlus  = x0.clone();
            double[] xMinus = x0.clone();
            xPlus[i]  += eps;
            xMinus[i] -= eps;

            double[] fPlus  = f(xPlus,  u0, m, g, l, I);
            double[] fMinus = f(xMinus, u0, m, g, l, I);

            for (int k = 0; k < 2; ++k) {
                A[k][i] = (fPlus[k] - fMinus[k]) / (2.0 * eps);
            }
        }

        // Column of B
        double uPlus  = u0 + eps;
        double uMinus = u0 - eps;
        double[] fPlusU  = f(x0, uPlus,  m, g, l, I);
        double[] fMinusU = f(x0, uMinus, m, g, l, I);
        for (int k = 0; k < 2; ++k) {
            B[k] = (fPlusU[k] - fMinusU[k]) / (2.0 * eps);
        }
    }

    public static void main(String[] args) {
        double[] x0 = {0.0, 0.0};
        double u0 = 0.0;
        double m = 1.0, g = 9.81, l = 1.0, I = 1.0;
        double eps = 1e-6;

        double[][] A = new double[2][2];
        double[] B = new double[2];

        linearize(x0, u0, m, g, l, I, eps, A, B);

        System.out.println("A:");
        for (int i = 0; i < 2; ++i) {
            System.out.println(A[i][0] + "  " + A[i][1]);
        }
        System.out.println("B:");
        System.out.println(B[0] + "  " + B[1]);
    }
}
