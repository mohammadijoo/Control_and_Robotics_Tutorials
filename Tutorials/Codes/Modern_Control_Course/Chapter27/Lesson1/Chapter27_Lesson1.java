// Chapter27_Lesson1.java
// Formulating Tracking Problems in State-Space Form
//
// Compile and run:
//   javac Chapter27_Lesson1.java
//   java Chapter27_Lesson1
//
// This from-scratch implementation solves the constant tracking equilibrium
// equations for a second-order plant and simulates a deviation-feedback
// tracking formulation using RK4 integration.

public class Chapter27_Lesson1 {
    static class Equilibrium {
        double x1Bar;
        double x2Bar;
        double uBar;
        double residualNorm;

        Equilibrium(double x1Bar, double x2Bar, double uBar, double residualNorm) {
            this.x1Bar = x1Bar;
            this.x2Bar = x2Bar;
            this.uBar = uBar;
            this.residualNorm = residualNorm;
        }
    }

    static double[] solve3x3(double[][] M, double[] b) {
        for (int k = 0; k < 3; k++) {
            int pivot = k;
            double best = Math.abs(M[k][k]);
            for (int i = k + 1; i < 3; i++) {
                if (Math.abs(M[i][k]) > best) {
                    best = Math.abs(M[i][k]);
                    pivot = i;
                }
            }

            if (pivot != k) {
                double[] tmpRow = M[k];
                M[k] = M[pivot];
                M[pivot] = tmpRow;

                double tmp = b[k];
                b[k] = b[pivot];
                b[pivot] = tmp;
            }

            double diag = M[k][k];
            if (Math.abs(diag) < 1e-12) {
                throw new RuntimeException("Singular equilibrium matrix.");
            }

            for (int j = k; j < 3; j++) {
                M[k][j] /= diag;
            }
            b[k] /= diag;

            for (int i = 0; i < 3; i++) {
                if (i == k) {
                    continue;
                }
                double factor = M[i][k];
                for (int j = k; j < 3; j++) {
                    M[i][j] -= factor * M[k][j];
                }
                b[i] -= factor * b[k];
            }
        }
        return b;
    }

    static Equilibrium computeEquilibrium(double reference, double disturbance) {
        // Plant:
        //   x1_dot = x2
        //   x2_dot = -2 x1 - 0.8 x2 + u + d
        //   y      = x1
        //
        // Constant tracking equilibrium:
        //   0 = x2
        //   0 = -2 x1 - 0.8 x2 + u + d
        //   r = x1

        double[][] M = {
            {0.0, 1.0, 0.0},
            {-2.0, -0.8, 1.0},
            {1.0, 0.0, 0.0}
        };
        double[] rhs = {0.0, -disturbance, reference};

        double[] theta = solve3x3(M, rhs);
        double x1 = theta[0];
        double x2 = theta[1];
        double u = theta[2];

        double res1 = x2;
        double res2 = -2.0 * x1 - 0.8 * x2 + u + disturbance;
        double res3 = x1 - reference;
        double norm = Math.sqrt(res1 * res1 + res2 * res2 + res3 * res3);

        return new Equilibrium(x1, x2, u, norm);
    }

    static double[] dynamics(double[] x, Equilibrium eq, double disturbance) {
        double k1 = 4.0;
        double k2 = 2.2;
        double u = eq.uBar - k1 * (x[0] - eq.x1Bar) - k2 * (x[1] - eq.x2Bar);

        return new double[] {
            x[1],
            -2.0 * x[0] - 0.8 * x[1] + u + disturbance
        };
    }

    static double[] addScaled(double[] x, double[] k, double scale) {
        return new double[] {x[0] + scale * k[0], x[1] + scale * k[1]};
    }

    static double[] rk4Step(double[] x, double h, Equilibrium eq, double disturbance) {
        double[] k1 = dynamics(x, eq, disturbance);
        double[] k2 = dynamics(addScaled(x, k1, 0.5 * h), eq, disturbance);
        double[] k3 = dynamics(addScaled(x, k2, 0.5 * h), eq, disturbance);
        double[] k4 = dynamics(addScaled(x, k3, h), eq, disturbance);

        return new double[] {
            x[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
            x[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        };
    }

    public static void main(String[] args) {
        double reference = 1.0;
        double disturbance = 0.2;
        Equilibrium eq = computeEquilibrium(reference, disturbance);

        System.out.printf("x_bar = [%.8f, %.8f]%n", eq.x1Bar, eq.x2Bar);
        System.out.printf("u_bar = %.8f%n", eq.uBar);
        System.out.printf("equilibrium residual norm = %.8e%n", eq.residualNorm);

        double[] x = {0.0, 0.0};
        double h = 0.001;
        double tf = 8.0;
        int steps = (int) (tf / h);

        for (int i = 0; i < steps; i++) {
            x = rk4Step(x, h, eq, disturbance);
        }

        double y = x[0];
        double e = y - reference;
        System.out.printf("final y(T) = %.8f%n", y);
        System.out.printf("final e(T) = %.8e%n", e);
    }
}
