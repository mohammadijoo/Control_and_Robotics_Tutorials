// Chapter23_Lesson1.java
// Formulating the SISO pole-placement problem for a second-order companion-form system.
// Compile: javac Chapter23_Lesson1.java
// Run:     java Chapter23_Lesson1

public class Chapter23_Lesson1 {
    static class Matrix2 {
        double a11, a12, a21, a22;
        Matrix2(double a11, double a12, double a21, double a22) {
            this.a11 = a11;
            this.a12 = a12;
            this.a21 = a21;
            this.a22 = a22;
        }
    }

    static double determinant(Matrix2 M) {
        return M.a11 * M.a22 - M.a12 * M.a21;
    }

    static void printEigenvalues2x2(Matrix2 M) {
        double trace = M.a11 + M.a22;
        double det = determinant(M);
        double disc = trace * trace - 4.0 * det;

        if (disc >= 0.0) {
            double root = Math.sqrt(disc);
            double lambda1 = (trace + root) / 2.0;
            double lambda2 = (trace - root) / 2.0;
            System.out.printf("closed-loop eigenvalue 1 = %.6f%n", lambda1);
            System.out.printf("closed-loop eigenvalue 2 = %.6f%n", lambda2);
        } else {
            double real = trace / 2.0;
            double imag = Math.sqrt(-disc) / 2.0;
            System.out.printf("closed-loop eigenvalue 1 = %.6f + %.6fi%n", real, imag);
            System.out.printf("closed-loop eigenvalue 2 = %.6f - %.6fi%n", real, imag);
        }
    }

    public static void main(String[] args) {
        // A = [[0, 1], [-2, -3]], b = [[0], [1]]
        Matrix2 A = new Matrix2(0.0, 1.0, -2.0, -3.0);
        double b1 = 0.0;
        double b2 = 1.0;

        // Controllability matrix C = [b, A b]
        Matrix2 Ctrb = new Matrix2(
            b1, A.a11 * b1 + A.a12 * b2,
            b2, A.a21 * b1 + A.a22 * b2
        );

        double detCtrb = determinant(Ctrb);
        System.out.printf("det(Ctrb) = %.6f%n", detCtrb);
        System.out.println("controllable = " + (Math.abs(detCtrb) > 1e-9));

        // Open-loop polynomial: s^2 + 3 s + 2.
        // Desired poles: -4 and -5, so desired polynomial: s^2 + 9 s + 20.
        double a0 = 2.0;
        double a1 = 3.0;
        double alpha0 = 20.0;
        double alpha1 = 9.0;

        // For companion form: K = [alpha0 - a0, alpha1 - a1].
        double k0 = alpha0 - a0;
        double k1 = alpha1 - a1;
        System.out.printf("K = [%.6f, %.6f]%n", k0, k1);

        // Acl = A - b K. Since b = [0, 1]^T, only the second row changes.
        Matrix2 Acl = new Matrix2(
            A.a11 - b1 * k0, A.a12 - b1 * k1,
            A.a21 - b2 * k0, A.a22 - b2 * k1
        );

        System.out.printf("A_cl = [[%.6f, %.6f], [%.6f, %.6f]]%n",
                          Acl.a11, Acl.a12, Acl.a21, Acl.a22);
        printEigenvalues2x2(Acl);

        // Characteristic polynomial of 2x2 matrix: s^2 - tr(Acl)s + det(Acl).
        double p1 = -(Acl.a11 + Acl.a22);
        double p0 = determinant(Acl);
        System.out.printf("closed-loop polynomial: s^2 + %.6f s + %.6f%n", p1, p0);
    }
}