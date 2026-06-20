/*
Chapter25_Lesson1.java
Uncontrollable Modes and Unassignable Poles

Compile:
    javac Chapter25_Lesson1.java
Run:
    java Chapter25_Lesson1
*/

public class Chapter25_Lesson1 {
    static class Mat2 {
        double a11, a12, a21, a22;
        Mat2(double a11, double a12, double a21, double a22) {
            this.a11 = a11; this.a12 = a12; this.a21 = a21; this.a22 = a22;
        }
    }

    static class Vec2 {
        double x1, x2;
        Vec2(double x1, double x2) { this.x1 = x1; this.x2 = x2; }
    }

    static Vec2 matVec(Mat2 A, Vec2 x) {
        return new Vec2(A.a11 * x.x1 + A.a12 * x.x2,
                        A.a21 * x.x1 + A.a22 * x.x2);
    }

    static double det2(double m11, double m12, double m21, double m22) {
        return m11 * m22 - m12 * m21;
    }

    static int rank2By2Columns(Vec2 c1, Vec2 c2, double tol) {
        double d = det2(c1.x1, c2.x1, c1.x2, c2.x2);
        if (Math.abs(d) > tol) return 2;
        if (Math.abs(c1.x1) > tol || Math.abs(c1.x2) > tol ||
            Math.abs(c2.x1) > tol || Math.abs(c2.x2) > tol) return 1;
        return 0;
    }

    static int pbhRank(double lambda, Mat2 A, Vec2 B, double tol) {
        // PBH matrix is [lambda I - A, B], a 2 x 3 matrix.
        double c1_1 = lambda - A.a11, c1_2 = -A.a21;
        double c2_1 = -A.a12,         c2_2 = lambda - A.a22;
        double c3_1 = B.x1,           c3_2 = B.x2;

        double m12 = det2(c1_1, c2_1, c1_2, c2_2);
        double m13 = det2(c1_1, c3_1, c1_2, c3_2);
        double m23 = det2(c2_1, c3_1, c2_2, c3_2);
        return (Math.abs(m12) > tol || Math.abs(m13) > tol || Math.abs(m23) > tol) ? 2 : 1;
    }

    static String eig2x2(Mat2 A) {
        double tr = A.a11 + A.a22;
        double det = A.a11 * A.a22 - A.a12 * A.a21;
        double disc = tr * tr - 4.0 * det;
        if (disc >= 0.0) {
            double s = Math.sqrt(disc);
            return String.format("{%.3f, %.3f}", (tr + s) / 2.0, (tr - s) / 2.0);
        } else {
            double real = tr / 2.0;
            double imag = Math.sqrt(-disc) / 2.0;
            return String.format("{%.3f + %.3fi, %.3f - %.3fi}", real, imag, real, imag);
        }
    }

    public static void main(String[] args) {
        Mat2 A = new Mat2(0.0, 0.0, 0.0, 2.0);
        Vec2 B = new Vec2(1.0, 0.0);
        Vec2 AB = matVec(A, B);
        int rankC = rank2By2Columns(B, AB, 1e-10);

        System.out.println("A = [[0, 0], [0, 2]], B = [1, 0]^T");
        System.out.printf("Controllability matrix C = [[%.3f, %.3f], [%.3f, %.3f]]%n",
                          B.x1, AB.x1, B.x2, AB.x2);
        System.out.println("rank(C) = " + rankC + " out of n = 2");

        System.out.println("\nPBH ranks:");
        for (double lambda : new double[] {0.0, 2.0}) {
            int r = pbhRank(lambda, A, B, 1e-10);
            System.out.printf("  lambda = %.3f, rank([lambda I - A, B]) = %d/2", lambda, r);
            if (r < 2) System.out.print("  <-- uncontrollable mode");
            System.out.println();
        }

        System.out.println("\nClosed-loop eigenvalues for K = [k1, k2]:");
        double[][] gains = {{0, 0}, {3, 0}, {8, 100}, {-1, -50}};
        for (double[] K : gains) {
            double k1 = K[0], k2 = K[1];
            Mat2 Acl = new Mat2(A.a11 - B.x1 * k1, A.a12 - B.x1 * k2,
                                A.a21 - B.x2 * k1, A.a22 - B.x2 * k2);
            System.out.printf("  K = [%.3f, %.3f] -> eig(A-BK) = %s%n", k1, k2, eig2x2(Acl));
        }

        System.out.println("\nThe eigenvalue 2 remains fixed for every K, so it is unassignable.");
    }
}
