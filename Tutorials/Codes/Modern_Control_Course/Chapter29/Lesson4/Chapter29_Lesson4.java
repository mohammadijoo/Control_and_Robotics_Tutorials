// Chapter29_Lesson4.java
// Numerical controllability and observability Gramians for a 2-state LTV system.
// Build example:
//   javac Chapter29_Lesson4.java
//   java Chapter29_Lesson4

public class Chapter29_Lesson4 {
    static double[][] zero2() {
        return new double[][]{{0.0, 0.0}, {0.0, 0.0}};
    }

    static double[][] eye2() {
        return new double[][]{{1.0, 0.0}, {0.0, 1.0}};
    }

    static double[][] A(double t) {
        return new double[][]{
            {0.0, 1.0},
            {-(2.0 + 0.4 * Math.sin(t)), -(0.25 + 0.10 * Math.cos(2.0 * t))}
        };
    }

    static double[] B(double t) {
        return new double[]{0.0, 1.0 + 0.25 * Math.sin(0.5 * t)};
    }

    static double[] C(double t) {
        return new double[]{1.0, 0.30 * Math.cos(t)};
    }

    static double[][] add(double[][] X, double[][] Y) {
        double[][] Z = zero2();
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                Z[i][j] = X[i][j] + Y[i][j];
        return Z;
    }

    static double[][] scale(double[][] X, double a) {
        double[][] Z = zero2();
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                Z[i][j] = a * X[i][j];
        return Z;
    }

    static double[][] mul(double[][] X, double[][] Y) {
        double[][] Z = zero2();
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j)
                for (int k = 0; k < 2; ++k)
                    Z[i][j] += X[i][k] * Y[k][j];
        return Z;
    }

    static double[][] trans(double[][] X) {
        return new double[][]{{X[0][0], X[1][0]}, {X[0][1], X[1][1]}};
    }

    static double det2(double[][] X) {
        return X[0][0] * X[1][1] - X[0][1] * X[1][0];
    }

    static double[][] inv2(double[][] X) {
        double d = det2(X);
        if (Math.abs(d) < 1e-14) {
            throw new ArithmeticException("Singular 2x2 matrix encountered.");
        }
        return new double[][]{
            { X[1][1] / d, -X[0][1] / d},
            {-X[1][0] / d,  X[0][0] / d}
        };
    }

    static double[][] outer(double[] u, double[] v) {
        return new double[][]{
            {u[0] * v[0], u[0] * v[1]},
            {u[1] * v[0], u[1] * v[1]}
        };
    }

    static double[][] phiDerivative(double t, double[][] Phi) {
        return mul(A(t), Phi);
    }

    static double[][] rk4StepPhi(double[][] Phi, double t, double h) {
        double[][] k1 = phiDerivative(t, Phi);
        double[][] k2 = phiDerivative(t + 0.5 * h, add(Phi, scale(k1, 0.5 * h)));
        double[][] k3 = phiDerivative(t + 0.5 * h, add(Phi, scale(k2, 0.5 * h)));
        double[][] k4 = phiDerivative(t + h, add(Phi, scale(k3, h)));

        double[][] sum = add(add(k1, scale(k2, 2.0)), add(scale(k3, 2.0), k4));
        return add(Phi, scale(sum, h / 6.0));
    }

    static void printMat(double[][] M) {
        System.out.printf("[%.8f, %.8f]%n", M[0][0], M[0][1]);
        System.out.printf("[%.8f, %.8f]%n", M[1][0], M[1][1]);
    }

    static double minEigSym2(double[][] M) {
        double a = M[0][0];
        double b = 0.5 * (M[0][1] + M[1][0]);
        double d = M[1][1];
        double tr = a + d;
        double disc = Math.sqrt((a - d) * (a - d) + 4.0 * b * b);
        return 0.5 * (tr - disc);
    }

    public static void main(String[] args) {
        final double t0 = 0.0;
        final double tf = 6.0;
        final int nSteps = 4000;
        final double h = (tf - t0) / nSteps;

        double[] times = new double[nSteps + 1];
        double[][][] phis = new double[nSteps + 1][2][2];

        double[][] Phi = eye2();
        phis[0] = Phi;
        times[0] = t0;

        for (int k = 0; k < nSteps; ++k) {
            double t = t0 + k * h;
            Phi = rk4StepPhi(Phi, t, h);
            phis[k + 1] = Phi;
            times[k + 1] = t + h;
        }

        double[][] PhiTfT0 = phis[nSteps];
        double[][] Wc = zero2();
        double[][] Wo = zero2();

        for (int k = 0; k <= nSteps; ++k) {
            double weight = (k == 0 || k == nSteps) ? 0.5 : 1.0;
            double s = times[k];

            double[][] PhiST0 = phis[k];
            double[][] PhiTfS = mul(PhiTfT0, inv2(PhiST0));

            double[] bs = B(s);
            double[] cs = C(s);

            double[][] wcIntegrand = mul(mul(PhiTfS, outer(bs, bs)), trans(PhiTfS));
            double[][] woIntegrand = mul(mul(trans(PhiST0), outer(cs, cs)), PhiST0);

            Wc = add(Wc, scale(wcIntegrand, weight * h));
            Wo = add(Wo, scale(woIntegrand, weight * h));
        }

        System.out.println("Controllability Gramian Wc:");
        printMat(Wc);
        System.out.println("det(Wc): " + det2(Wc));
        System.out.println("min eig approx Wc: " + minEigSym2(Wc));

        System.out.println("\nObservability Gramian Wo:");
        printMat(Wo);
        System.out.println("det(Wo): " + det2(Wo));
        System.out.println("min eig approx Wo: " + minEigSym2(Wo));
    }
}
