/*
Chapter22_Lesson2.java
Closed-Loop State Matrix and Mode Relocation

This Java example implements the 2x2 SISO coefficient-matching calculation
from scratch. In larger projects, EJML or Apache Commons Math is recommended
for reliable linear algebra.

Compile:
    javac Chapter22_Lesson2.java

Run:
    java Chapter22_Lesson2
*/

public class Chapter22_Lesson2 {
    static class Matrix2 {
        double a11, a12, a21, a22;

        Matrix2(double a11, double a12, double a21, double a22) {
            this.a11 = a11;
            this.a12 = a12;
            this.a21 = a21;
            this.a22 = a22;
        }
    }

    static class Vector2 {
        double v1, v2;

        Vector2(double v1, double v2) {
            this.v1 = v1;
            this.v2 = v2;
        }
    }

    static class Complex {
        double re, im;

        Complex(double re, double im) {
            this.re = re;
            this.im = im;
        }

        @Override
        public String toString() {
            if (Math.abs(im) < 1e-12) {
                return String.format("%.6f", re);
            }
            return String.format("%.6f%+.6fi", re, im);
        }
    }

    static Matrix2 closedLoopMatrix(Matrix2 A, Vector2 B, Vector2 K) {
        // Acl = A - B K, with B 2x1 and K 1x2.
        return new Matrix2(
            A.a11 - B.v1 * K.v1, A.a12 - B.v1 * K.v2,
            A.a21 - B.v2 * K.v1, A.a22 - B.v2 * K.v2
        );
    }

    static double trace(Matrix2 M) {
        return M.a11 + M.a22;
    }

    static double determinant(Matrix2 M) {
        return M.a11 * M.a22 - M.a12 * M.a21;
    }

    static double[] characteristicCoefficients(Matrix2 M) {
        // det(sI - M) = s^2 - tr(M)s + det(M) = s^2 + a1 s + a0.
        return new double[]{-trace(M), determinant(M)};
    }

    static Complex[] eigenvalues2x2(Matrix2 M) {
        double tr = trace(M);
        double det = determinant(M);
        double disc = tr * tr - 4.0 * det;

        if (disc >= 0.0) {
            double root = Math.sqrt(disc);
            return new Complex[]{
                new Complex(0.5 * (tr + root), 0.0),
                new Complex(0.5 * (tr - root), 0.0)
            };
        }

        double root = Math.sqrt(-disc);
        return new Complex[]{
            new Complex(0.5 * tr, 0.5 * root),
            new Complex(0.5 * tr, -0.5 * root)
        };
    }

    static Vector2 solve2x2(double m11, double m12, double m21, double m22,
                            double b1, double b2) {
        double det = m11 * m22 - m12 * m21;
        if (Math.abs(det) < 1e-12) {
            throw new IllegalArgumentException("Singular coefficient-matching equations.");
        }

        return new Vector2(
            ( b1 * m22 - m12 * b2) / det,
            ( m11 * b2 - b1 * m21) / det
        );
    }

    static Vector2 secondOrderFeedbackByMatching(Matrix2 A, Vector2 B,
                                                 double p1, double p2) {
        double desiredA1 = -(p1 + p2);
        double desiredA0 = p1 * p2;

        double[] c00 = characteristicCoefficients(closedLoopMatrix(A, B, new Vector2(0.0, 0.0)));
        double[] c10 = characteristicCoefficients(closedLoopMatrix(A, B, new Vector2(1.0, 0.0)));
        double[] c01 = characteristicCoefficients(closedLoopMatrix(A, B, new Vector2(0.0, 1.0)));

        // [a1(k); a0(k)] = c00 + M [k1; k2]
        double m11 = c10[0] - c00[0];
        double m21 = c10[1] - c00[1];
        double m12 = c01[0] - c00[0];
        double m22 = c01[1] - c00[1];

        double rhs1 = desiredA1 - c00[0];
        double rhs2 = desiredA0 - c00[1];

        return solve2x2(m11, m12, m21, m22, rhs1, rhs2);
    }

    public static void main(String[] args) {
        Matrix2 A = new Matrix2(0.0, 1.0, -2.0, -0.4);
        Vector2 B = new Vector2(0.0, 1.0);

        Complex[] openModes = eigenvalues2x2(A);
        System.out.println("Open-loop modes:");
        System.out.println("lambda1 = " + openModes[0]);
        System.out.println("lambda2 = " + openModes[1]);

        double p1 = -2.0;
        double p2 = -3.0;

        Vector2 K = secondOrderFeedbackByMatching(A, B, p1, p2);
        Matrix2 Acl = closedLoopMatrix(A, B, K);
        Complex[] closedModes = eigenvalues2x2(Acl);
        double[] coeffs = characteristicCoefficients(Acl);

        System.out.printf("%nK = [%.6f, %.6f]%n", K.v1, K.v2);
        System.out.printf("Acl = [[%.6f, %.6f], [%.6f, %.6f]]%n",
                          Acl.a11, Acl.a12, Acl.a21, Acl.a22);
        System.out.println("Closed-loop modes:");
        System.out.println("lambda1 = " + closedModes[0]);
        System.out.println("lambda2 = " + closedModes[1]);
        System.out.printf("Characteristic polynomial: s^2 + %.6f s + %.6f%n",
                          coeffs[0], coeffs[1]);
    }
}
