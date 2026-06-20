// Chapter9_Lesson1.java
// Modern Control — Chapter 9, Lesson 1
// Stability, Asymptotic Stability, and Instability (State-Space View)
//
// This self-contained example handles 2-by-2 continuous-time systems.
// It classifies by trace/determinant eigenvalue information and simulates
// x_dot = A x using RK4.

public class Chapter9_Lesson1 {

    static class Matrix2 {
        double a11, a12, a21, a22;

        Matrix2(double a11, double a12, double a21, double a22) {
            this.a11 = a11;
            this.a12 = a12;
            this.a21 = a21;
            this.a22 = a22;
        }

        double trace() {
            return a11 + a22;
        }

        double det() {
            return a11 * a22 - a12 * a21;
        }

        double[] multiply(double[] x) {
            return new double[] {
                a11 * x[0] + a12 * x[1],
                a21 * x[0] + a22 * x[1]
            };
        }
    }

    static double norm(double[] x) {
        return Math.sqrt(x[0] * x[0] + x[1] * x[1]);
    }

    static double[] add(double[] x, double[] y, double scale) {
        return new double[] {x[0] + scale * y[0], x[1] + scale * y[1]};
    }

    static double[] rk4Step(Matrix2 A, double[] x, double h) {
        double[] k1 = A.multiply(x);
        double[] k2 = A.multiply(add(x, k1, 0.5 * h));
        double[] k3 = A.multiply(add(x, k2, 0.5 * h));
        double[] k4 = A.multiply(add(x, k3, h));

        return new double[] {
            x[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
            x[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        };
    }

    static String classify2x2(Matrix2 A, double tol) {
        double tr = A.trace();
        double det = A.det();
        double disc = tr * tr - 4.0 * det;

        if (disc >= 0.0) {
            double sqrtDisc = Math.sqrt(disc);
            double lambda1 = 0.5 * (tr + sqrtDisc);
            double lambda2 = 0.5 * (tr - sqrtDisc);

            if (lambda1 < -tol && lambda2 < -tol) return "asymptotically stable";
            if (lambda1 > tol || lambda2 > tol) return "unstable";
            return "borderline: inspect semisimplicity";
        } else {
            double realPart = 0.5 * tr;
            if (realPart < -tol) return "asymptotically stable";
            if (realPart > tol) return "unstable";
            return "marginal center candidate";
        }
    }

    static void reportSystem(String name, Matrix2 A, double[] x0) {
        System.out.println("\n=== " + name + " ===");
        System.out.printf("A = [[%.3f, %.3f], [%.3f, %.3f]]%n", A.a11, A.a12, A.a21, A.a22);
        System.out.printf("trace = %.6f, determinant = %.6f%n", A.trace(), A.det());
        System.out.println("classification = " + classify2x2(A, 1e-10));

        double h = 0.01;
        int steps = 1000;
        double[] x = new double[] {x0[0], x0[1]};
        double maxNorm = norm(x);

        for (int k = 0; k < steps; k++) {
            x = rk4Step(A, x, h);
            maxNorm = Math.max(maxNorm, norm(x));
        }

        System.out.printf("||x(0)|| = %.6f%n", norm(x0));
        System.out.printf("||x(10)|| approx = %.6f%n", norm(x));
        System.out.printf("max ||x(t)|| approx on [0,10] = %.6f%n", maxNorm);
    }

    public static void main(String[] args) {
        double[] x0 = {1.0, -0.5};

        Matrix2 stable = new Matrix2(-1.0, 2.0, -3.0, -2.0);
        Matrix2 center = new Matrix2(0.0, 1.0, -1.0, 0.0);
        Matrix2 unstable = new Matrix2(0.2, 1.0, 0.0, -1.0);

        reportSystem("Stable spiral", stable, x0);
        reportSystem("Marginal center", center, x0);
        reportSystem("Unstable saddle/source component", unstable, x0);
    }
}
