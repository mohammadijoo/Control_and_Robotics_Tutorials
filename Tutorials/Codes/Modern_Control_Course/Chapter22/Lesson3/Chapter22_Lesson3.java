/*
Chapter22_Lesson3.java
State Feedback vs Output Feedback (Concept Only)

Compile: javac Chapter22_Lesson3.java
Run:     java Chapter22_Lesson3
*/

public class Chapter22_Lesson3 {
    static class Matrix2 {
        double a11, a12, a21, a22;
        Matrix2(double a11, double a12, double a21, double a22) {
            this.a11 = a11;
            this.a12 = a12;
            this.a21 = a21;
            this.a22 = a22;
        }
    }

    static class Complex {
        double re, im;
        Complex(double re, double im) { this.re = re; this.im = im; }
        public String toString() {
            if (Math.abs(im) < 1e-12) return String.format("%.6f", re);
            return String.format("%.6f%+.6fi", re, im);
        }
    }

    static Complex[] eigenvalues2x2(Matrix2 m) {
        double tr = m.a11 + m.a22;
        double det = m.a11 * m.a22 - m.a12 * m.a21;
        double disc = tr * tr - 4.0 * det;
        if (disc >= 0.0) {
            double root = Math.sqrt(disc);
            return new Complex[] { new Complex((tr + root) / 2.0, 0.0), new Complex((tr - root) / 2.0, 0.0) };
        }
        double root = Math.sqrt(-disc);
        return new Complex[] { new Complex(tr / 2.0, root / 2.0), new Complex(tr / 2.0, -root / 2.0) };
    }

    static void printSummary(Matrix2 m, String name) {
        Complex[] lambda = eigenvalues2x2(m);
        System.out.printf("%s = [[%.3f, %.3f], [%.3f, %.3f]]%n", name, m.a11, m.a12, m.a21, m.a22);
        System.out.printf("eigenvalues(%s) = %s, %s%n", name, lambda[0], lambda[1]);
        System.out.printf("stable? %s%n%n", (lambda[0].re < 0.0 && lambda[1].re < 0.0));
    }

    public static void main(String[] args) {
        // Plant: x_dot = A x + B u, y = C x
        Matrix2 A = new Matrix2(0.0, 1.0, -2.0, -0.4);

        // B = [0; 1]. State feedback K = [k1 k2].
        double k1 = 4.0;
        double k2 = 2.6;
        Matrix2 AState = new Matrix2(A.a11, A.a12, A.a21 - k1, A.a22 - k2);

        // Static output feedback with y = position = [1 0] x and F = [f].
        // Effective gain F*C = [f 0], so velocity feedback is impossible here.
        double f = 4.0;
        Matrix2 AOutputPosition = new Matrix2(A.a11, A.a12, A.a21 - f, A.a22);

        // Full output y = x makes static output feedback equivalent to state feedback.
        Matrix2 AOutputFull = AState;

        printSummary(A, "A_open_loop");
        printSummary(AState, "A_state_feedback");
        printSummary(AOutputPosition, "A_static_output_feedback_position_only");
        printSummary(AOutputFull, "A_static_output_feedback_full_output");

        System.out.println("Geometry note:");
        System.out.println("For C = [1 0], every static output gain has K_eff = F*C = [f 0].");
        System.out.println("Therefore K = [4 2.6] is not implementable from position alone.");
    }
}
