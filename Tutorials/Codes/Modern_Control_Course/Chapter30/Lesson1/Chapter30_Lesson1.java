/*
Chapter30_Lesson1.java
Index-1 descriptor system reduction for E xdot = A x + B u.
Compile: javac Chapter30_Lesson1.java
Run:     java Chapter30_Lesson1
*/

public class Chapter30_Lesson1 {
    static final double[][] Ar = {
        {0.0, 1.0},
        {-3.0, -3.0}
    };
    static final double[] Br = {0.0, 2.0};

    static double uOfT(double t) {
        return t >= 1.0 ? 1.0 : 0.0;
    }

    static double[] f(double t, double[] x) {
        double u = uOfT(t);
        return new double[] {
            Ar[0][0] * x[0] + Ar[0][1] * x[1] + Br[0] * u,
            Ar[1][0] * x[0] + Ar[1][1] * x[1] + Br[1] * u
        };
    }

    static double[] addScaled(double[] x, double[] k, double scale) {
        return new double[] {x[0] + scale * k[0], x[1] + scale * k[1]};
    }

    static double[] rk4Step(double t, double[] x, double h) {
        double[] k1 = f(t, x);
        double[] k2 = f(t + 0.5 * h, addScaled(x, k1, 0.5 * h));
        double[] k3 = f(t + 0.5 * h, addScaled(x, k2, 0.5 * h));
        double[] k4 = f(t + h, addScaled(x, k3, h));
        return new double[] {
            x[0] + (h / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
            x[1] + (h / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
        };
    }

    static double algebraicX3(double[] xd, double u) {
        // Constraint: 0 = x1 + x3 - u, hence x3 = -x1 + u.
        return -xd[0] + u;
    }

    static void printReducedEigenvalues() {
        double trace = Ar[0][0] + Ar[1][1];
        double determinant = Ar[0][0] * Ar[1][1] - Ar[0][1] * Ar[1][0];
        double discriminant = trace * trace - 4.0 * determinant;
        if (discriminant >= 0.0) {
            double r1 = 0.5 * (trace + Math.sqrt(discriminant));
            double r2 = 0.5 * (trace - Math.sqrt(discriminant));
            System.out.println("Reduced eigenvalues: " + r1 + ", " + r2);
        } else {
            double real = 0.5 * trace;
            double imag = 0.5 * Math.sqrt(-discriminant);
            System.out.println("Reduced eigenvalues: " + real + " + " + imag + "i, "
                    + real + " - " + imag + "i");
        }
    }

    public static void main(String[] args) {
        printReducedEigenvalues();
        double[] xd = {0.2, 0.0};
        double h = 0.01;
        int steps = 800;

        System.out.println("time,x1,x2,x3,constraint_residual");
        for (int k = 0; k <= steps; k++) {
            double t = k * h;
            double u = uOfT(t);
            double x3 = algebraicX3(xd, u);
            double residual = xd[0] + x3 - u;
            if (k % 40 == 0) {
                System.out.printf("%.6f,%.6f,%.6f,%.6f,%.6e%n", t, xd[0], xd[1], x3, residual);
            }
            xd = rk4Step(t, xd, h);
        }
    }
}
