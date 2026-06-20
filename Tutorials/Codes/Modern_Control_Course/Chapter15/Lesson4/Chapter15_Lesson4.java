// Chapter15_Lesson4.java
// Condition Numbers and Sensitivity in State Reconstruction
//
// Build:
//   javac Chapter15_Lesson4.java
// Run:
//   java Chapter15_Lesson4
//
// This example uses a minimal from-scratch 2x2 implementation.
// For larger Modern Control workflows in Java, consider EJML, JBLAS,
// Apache Commons Math, or ND4J for matrix algebra.

public class Chapter15_Lesson4 {
    static class Vec2 {
        double x1, x2;
        Vec2(double x1, double x2) {
            this.x1 = x1;
            this.x2 = x2;
        }
    }

    static class Mat2 {
        double a11, a12, a21, a22;
        Mat2(double a11, double a12, double a21, double a22) {
            this.a11 = a11;
            this.a12 = a12;
            this.a21 = a21;
            this.a22 = a22;
        }
    }

    static double det2(Mat2 M) {
        return M.a11 * M.a22 - M.a12 * M.a21;
    }

    static Vec2 solve2(Mat2 M, Vec2 b) {
        double d = det2(M);
        return new Vec2(
            ( M.a22 * b.x1 - M.a12 * b.x2) / d,
            (-M.a21 * b.x1 + M.a11 * b.x2) / d
        );
    }

    static double[] eigenvaluesSymmetric2(Mat2 M) {
        double tr = M.a11 + M.a22;
        double diff = M.a11 - M.a22;
        double rad = Math.sqrt(diff * diff + 4.0 * M.a12 * M.a12);
        return new double[] {0.5 * (tr - rad), 0.5 * (tr + rad)};
    }

    public static void main(String[] args) {
        double eps = 0.02;
        double T = 5.0;
        int N = 200000;
        double dt = T / N;

        Vec2 x0True = new Vec2(1.0, -1.5);

        Mat2 W = new Mat2(0.0, 0.0, 0.0, 0.0);
        Vec2 b = new Vec2(0.0, 0.0);

        for (int k = 0; k < N; k++) {
            double t = (k + 0.5) * dt;

            double phi1 = Math.exp(-t);
            double phi2 = Math.exp(-2.0 * t);

            // H(t) = C exp(A t) = [phi1, eps phi2]
            double h1 = phi1;
            double h2 = eps * phi2;

            double yClean = h1 * x0True.x1 + h2 * x0True.x2;
            double noise = 1.0e-3 * Math.sin(37.0 * t);
            double y = yClean + noise;

            W.a11 += h1 * h1 * dt;
            W.a12 += h1 * h2 * dt;
            W.a21 += h2 * h1 * dt;
            W.a22 += h2 * h2 * dt;

            b.x1 += h1 * y * dt;
            b.x2 += h2 * y * dt;
        }

        Vec2 xhat = solve2(W, b);
        double[] evals = eigenvaluesSymmetric2(W);
        double kappa = evals[1] / evals[0];

        System.out.printf("W = [[%.12f, %.12f], [%.12f, %.12f]]%n",
            W.a11, W.a12, W.a21, W.a22);
        System.out.printf("lambda_min = %.12e%n", evals[0]);
        System.out.printf("lambda_max = %.12e%n", evals[1]);
        System.out.printf("condition number = %.12e%n%n", kappa);

        System.out.printf("true x0 = [%.12f, %.12f]%n", x0True.x1, x0True.x2);
        System.out.printf("estimated x0 = [%.12f, %.12f]%n", xhat.x1, xhat.x2);

        // Ridge-regularized reconstruction: (W + alpha I) x = b
        double alpha = 1.0e-5;
        Mat2 Wr = new Mat2(W.a11 + alpha, W.a12, W.a21, W.a22 + alpha);
        Vec2 xr = solve2(Wr, b);
        System.out.printf("ridge estimated x0 = [%.12f, %.12f]%n", xr.x1, xr.x2);
    }
}
