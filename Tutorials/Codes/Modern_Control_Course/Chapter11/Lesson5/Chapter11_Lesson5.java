/*
Chapter11_Lesson5.java
Java RK4 computation of the finite-horizon LTV controllability Gramian.
Compile and run:
    javac Chapter11_Lesson5.java
    java Chapter11_Lesson5
*/

public class Chapter11_Lesson5 {
    static double[][] A(double t) {
        return new double[][]{
            {0.0, 1.0},
            {-(2.0 + 0.60 * Math.sin(t)), -0.25}
        };
    }

    static double[] B(double t) {
        return new double[]{0.0, 1.0 + 0.45 * Math.cos(t)};
    }

    static double[][] mul(double[][] X, double[][] Y) {
        double[][] Z = new double[2][2];
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                for (int k = 0; k < 2; k++)
                    Z[i][j] += X[i][k] * Y[k][j];
        return Z;
    }

    static double[][] transpose(double[][] X) {
        return new double[][]{{X[0][0], X[1][0]}, {X[0][1], X[1][1]}};
    }

    static double[] derivative(double t, double[] z) {
        double[][] Phi = {{z[0], z[1]}, {z[2], z[3]}};
        double[][] W = {{z[4], z[5]}, {z[6], z[7]}};
        double[][] At = A(t);
        double[] Bt = B(t);
        double[][] dPhi = mul(At, Phi);
        double[][] AW = mul(At, W);
        double[][] WAT = mul(W, transpose(At));

        double[][] dW = new double[2][2];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                dW[i][j] = AW[i][j] + WAT[i][j] + Bt[i] * Bt[j];
            }
        }
        return new double[]{dPhi[0][0], dPhi[0][1], dPhi[1][0], dPhi[1][1],
                            dW[0][0], dW[0][1], dW[1][0], dW[1][1]};
    }

    static double[] rk4Step(double t, double[] z, double h) {
        double[] k1 = derivative(t, z);
        double[] z2 = new double[8], z3 = new double[8], z4 = new double[8];
        for (int i = 0; i < 8; i++) z2[i] = z[i] + 0.5 * h * k1[i];
        double[] k2 = derivative(t + 0.5 * h, z2);
        for (int i = 0; i < 8; i++) z3[i] = z[i] + 0.5 * h * k2[i];
        double[] k3 = derivative(t + 0.5 * h, z3);
        for (int i = 0; i < 8; i++) z4[i] = z[i] + h * k3[i];
        double[] k4 = derivative(t + h, z4);

        double[] zn = new double[8];
        for (int i = 0; i < 8; i++)
            zn[i] = z[i] + h * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
        return zn;
    }

    public static void main(String[] args) {
        double t0 = 0.0, tf = 4.0;
        int steps = 4000;
        double h = (tf - t0) / steps;
        double[] z = {1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0};
        double t = t0;
        for (int k = 0; k < steps; k++) {
            z = rk4Step(t, z, h);
            t += h;
        }

        double w00 = z[4];
        double w01 = 0.5 * (z[5] + z[6]);
        double w11 = z[7];
        double trace = w00 + w11;
        double det = w00 * w11 - w01 * w01;
        double disc = Math.sqrt(Math.max(0.0, trace * trace - 4.0 * det));
        double lambdaMin = 0.5 * (trace - disc);

        System.out.printf("Wc(0,4) =%n%.10f  %.10f%n%.10f  %.10f%n", w00, w01, w01, w11);
        System.out.printf("det(Wc) = %.10f%n", det);
        System.out.printf("lambda_min(Wc) = %.10f%n", lambdaMin);
        System.out.println("Controllable? " + (lambdaMin > 1e-8 ? "yes" : "no"));
    }
}
