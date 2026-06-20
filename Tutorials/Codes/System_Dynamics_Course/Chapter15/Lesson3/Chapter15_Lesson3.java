// Chapter15_Lesson3.java
// Backward Euler with Newton iterations for a scalar nonlinear stiff ODE:
//   y' = -1000 (y - cos t) - sin t
// Exact solution y(t) = cos t for consistent initial condition y(0)=1.

public class Chapter15_Lesson3 {

    static double f(double t, double y) {
        return -1000.0 * (y - Math.cos(t)) - Math.sin(t);
    }

    static double dfdY(double t, double y) {
        return -1000.0;
    }

    static double backwardEulerNewtonStep(double tNext, double yPrev, double h) {
        double y = yPrev; // initial guess
        int maxIter = 20;
        double tol = 1e-12;

        for (int k = 0; k < maxIter; k++) {
            double F = y - yPrev - h * f(tNext, y);
            double dF = 1.0 - h * dfdY(tNext, y);
            double delta = -F / dF;
            y += delta;
            if (Math.abs(delta) < tol) {
                break;
            }
        }
        return y;
    }

    public static void main(String[] args) {
        double h = 0.05;
        int N = 40;
        double y = 1.0;

        System.out.println("Backward Euler + Newton (Java)");
        System.out.printf("%5s %12s %18s %18s%n", "n", "t", "y_num", "y_exact");

        for (int n = 1; n <= N; n++) {
            double tNext = n * h;
            y = backwardEulerNewtonStep(tNext, y, h);
            double yExact = Math.cos(tNext);
            System.out.printf("%5d %12.6f %18.10f %18.10f%n", n, tNext, y, yExact);
        }

        // For production Java libraries, consider Hipparchus ODE solvers for broader method support.
    }
}
