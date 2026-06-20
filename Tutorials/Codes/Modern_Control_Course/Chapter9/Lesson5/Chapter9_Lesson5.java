/*
Chapter9_Lesson5.java

Modern Control — Chapter 9, Lesson 5
Examples of Stable, Marginal, and Unstable State Matrices

This Java program classifies 2x2 real continuous-time matrices
A for x_dot = A x using the closed-form eigenvalues of a 2x2 matrix.

Compile:
    javac Chapter9_Lesson5.java
Run:
    java Chapter9_Lesson5
*/

public class Chapter9_Lesson5 {
    static class Matrix2 {
        final double a11, a12, a21, a22;

        Matrix2(double a11, double a12, double a21, double a22) {
            this.a11 = a11;
            this.a12 = a12;
            this.a21 = a21;
            this.a22 = a22;
        }
    }

    static class Complex {
        final double re, im;

        Complex(double re, double im) {
            this.re = re;
            this.im = im;
        }

        Complex plus(Complex other) {
            return new Complex(this.re + other.re, this.im + other.im);
        }

        Complex minus(Complex other) {
            return new Complex(this.re - other.re, this.im - other.im);
        }

        Complex scale(double s) {
            return new Complex(s * this.re, s * this.im);
        }

        double abs() {
            return Math.hypot(re, im);
        }

        static Complex sqrtReal(double x) {
            if (x >= 0.0) {
                return new Complex(Math.sqrt(x), 0.0);
            }
            return new Complex(0.0, Math.sqrt(-x));
        }

        @Override
        public String toString() {
            if (im >= 0.0) {
                return String.format("%.6f + %.6fi", re, im);
            }
            return String.format("%.6f - %.6fi", re, -im);
        }
    }

    static Complex[] eigenvalues2x2(Matrix2 A) {
        double tr = A.a11 + A.a22;
        double det = A.a11 * A.a22 - A.a12 * A.a21;
        double disc = tr * tr - 4.0 * det;
        Complex root = Complex.sqrtReal(disc);
        Complex center = new Complex(tr, 0.0).scale(0.5);
        return new Complex[] {
            center.plus(root.scale(0.5)),
            center.minus(root.scale(0.5))
        };
    }

    static double determinant(Matrix2 A) {
        return A.a11 * A.a22 - A.a12 * A.a21;
    }

    static int rank2x2(Matrix2 A, double tol) {
        if (Math.abs(A.a11) < tol && Math.abs(A.a12) < tol &&
            Math.abs(A.a21) < tol && Math.abs(A.a22) < tol) {
            return 0;
        }
        if (Math.abs(determinant(A)) > tol) {
            return 2;
        }
        return 1;
    }

    static String classify2x2(Matrix2 A) {
        double tol = 1e-9;
        Complex[] lambda = eigenvalues2x2(A);
        double maxReal = Math.max(lambda[0].re, lambda[1].re);

        if (maxReal < -tol) {
            return "asymptotically stable";
        }
        if (maxReal > tol) {
            return "unstable";
        }

        // Boundary case: repeated eigenvalue on the imaginary axis can be defective.
        if (lambda[0].minus(lambda[1]).abs() < 1e-8 && Math.abs(lambda[0].re) < 1e-8) {
            Matrix2 shifted = new Matrix2(
                A.a11 - lambda[0].re, A.a12,
                A.a21, A.a22 - lambda[0].re
            );
            int geometricMultiplicity = 2 - rank2x2(shifted, 1e-10);
            if (geometricMultiplicity < 2) {
                return "unstable";
            }
        }

        return "marginally stable";
    }

    static void report(String name, Matrix2 A) {
        Complex[] lambda = eigenvalues2x2(A);
        System.out.println("\n" + name);
        System.out.println("-".repeat(name.length()));
        System.out.printf("A = [[%.3f, %.3f], [%.3f, %.3f]]%n", A.a11, A.a12, A.a21, A.a22);
        System.out.println("lambda1 = " + lambda[0]);
        System.out.println("lambda2 = " + lambda[1]);
        System.out.println("classification = " + classify2x2(A));
    }

    public static void main(String[] args) {
        String[] names = {
            "Stable diagonal",
            "Stable damped oscillator",
            "Stable but nonnormal",
            "Marginal oscillator",
            "Marginal with semisimple zero",
            "Unstable positive eigenvalue",
            "Unstable defective zero"
        };

        Matrix2[] matrices = {
            new Matrix2(-2.0, 0.0, 0.0, -5.0),
            new Matrix2(0.0, 1.0, -4.0, -2.0),
            new Matrix2(-1.0, 50.0, 0.0, -2.0),
            new Matrix2(0.0, 1.0, -1.0, 0.0),
            new Matrix2(0.0, 0.0, 0.0, -2.0),
            new Matrix2(1.0, 0.0, 0.0, -2.0),
            new Matrix2(0.0, 1.0, 0.0, 0.0)
        };

        for (int i = 0; i < names.length; i++) {
            report(names[i], matrices[i]);
        }
    }
}
