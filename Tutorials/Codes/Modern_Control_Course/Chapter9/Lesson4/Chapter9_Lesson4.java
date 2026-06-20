/*
Chapter9_Lesson4.java

Compile and run:
    javac Chapter9_Lesson4.java
    java Chapter9_Lesson4

The program demonstrates that stable input-output behavior does not imply
internal stability when the realization contains a hidden unstable mode.
*/

import java.util.function.DoubleUnaryOperator;

public class Chapter9_Lesson4 {
    static class State {
        double x1;
        double x2;

        State(double x1, double x2) {
            this.x1 = x1;
            this.x2 = x2;
        }
    }

    static State derivative(State x, double u) {
        return new State(-x.x1 + u, x.x2);
    }

    static State addScaled(State x, State k, double scale) {
        return new State(x.x1 + scale * k.x1, x.x2 + scale * k.x2);
    }

    static State rk4Step(State x, double u, double h) {
        State k1 = derivative(x, u);
        State k2 = derivative(addScaled(x, k1, 0.5 * h), u);
        State k3 = derivative(addScaled(x, k2, 0.5 * h), u);
        State k4 = derivative(addScaled(x, k3, h), u);

        double x1Next = x.x1 + (h / 6.0) * (k1.x1 + 2.0 * k2.x1 + 2.0 * k3.x1 + k4.x1);
        double x2Next = x.x2 + (h / 6.0) * (k1.x2 + 2.0 * k2.x2 + 2.0 * k3.x2 + k4.x2);
        return new State(x1Next, x2Next);
    }

    static double output(State x, double u) {
        return x.x1;
    }

    static double norm(State x) {
        return Math.sqrt(x.x1 * x.x1 + x.x2 * x.x2);
    }

    static void simulate(State x0,
                         DoubleUnaryOperator uFun,
                         double tFinal,
                         double h,
                         String caseName) {
        State x = new State(x0.x1, x0.x2);
        double maxAbsY = 0.0;
        double maxNormX = norm(x);
        int nSteps = (int)(tFinal / h);

        for (int k = 0; k < nSteps; k++) {
            double t = k * h;
            double u = uFun.applyAsDouble(t);
            double y = output(x, u);
            maxAbsY = Math.max(maxAbsY, Math.abs(y));
            maxNormX = Math.max(maxNormX, norm(x));
            x = rk4Step(x, u, h);
        }

        double yFinal = output(x, uFun.applyAsDouble(tFinal));
        maxAbsY = Math.max(maxAbsY, Math.abs(yFinal));
        maxNormX = Math.max(maxNormX, norm(x));

        System.out.println("\n" + caseName);
        System.out.printf("  final x1 = %.6f, final x2 = %.6f%n", x.x1, x.x2);
        System.out.printf("  final y  = %.6f%n", yFinal);
        System.out.printf("  max |y|  = %.6f%n", maxAbsY);
        System.out.printf("  max ||x||= %.6f%n", maxNormX);
    }

    public static void main(String[] args) {
        System.out.println("A eigenvalues are -1 and +1.");
        System.out.println("Internal stability: unstable because one eigenvalue has positive real part.");
        System.out.println("External transfer function from u to y: G(s) = 1/(s + 1), externally stable.");

        simulate(new State(0.0, 1.0),
                 t -> 0.0,
                 5.0,
                 0.01,
                 "Case 1: zero input, hidden unstable initial condition x(0)=[0,1]^T");

        simulate(new State(0.0, 0.0),
                 t -> 1.0,
                 5.0,
                 0.01,
                 "Case 2: unit step input, zero initial state");
    }
}
