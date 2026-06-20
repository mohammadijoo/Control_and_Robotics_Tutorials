// Chapter21_Lesson3.java
// Zero Dynamics and Internal System Behavior
//
// Compile and run:
//   javac Chapter21_Lesson3.java
//   java Chapter21_Lesson3
//
// This plain-Java implementation integrates the zero dynamics for
//   G(s) = (s - 1) / ((s + 1)(s + 2)(s + 3)).
// On the output-nulling manifold x2=x1, x3=x1, the required input is u=18*x1.

public class Chapter21_Lesson3 {
    static final double[][] A = {
            {0.0, 1.0, 0.0},
            {0.0, 0.0, 1.0},
            {-6.0, -11.0, -6.0}
    };

    static final double[] B = {0.0, 0.0, 1.0};
    static final double[] C = {-1.0, 1.0, 0.0};

    static double outputNullingInput(double[] x) {
        return 18.0 * x[0];
    }

    static double[] dynamics(double[] x) {
        double u = outputNullingInput(x);
        double[] dx = new double[3];

        for (int i = 0; i < 3; i++) {
            dx[i] = B[i] * u;
            for (int j = 0; j < 3; j++) {
                dx[i] += A[i][j] * x[j];
            }
        }
        return dx;
    }

    static double output(double[] x) {
        return C[0] * x[0] + C[1] * x[1] + C[2] * x[2];
    }

    static double[] add(double[] x, double scale, double[] v) {
        return new double[] {
                x[0] + scale * v[0],
                x[1] + scale * v[1],
                x[2] + scale * v[2]
        };
    }

    static double[] rk4Step(double[] x, double h) {
        double[] k1 = dynamics(x);
        double[] k2 = dynamics(add(x, 0.5 * h, k1));
        double[] k3 = dynamics(add(x, 0.5 * h, k2));
        double[] k4 = dynamics(add(x, h, k3));

        return new double[] {
                x[0] + h * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0,
                x[1] + h * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0,
                x[2] + h * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0
        };
    }

    public static void main(String[] args) {
        System.out.println("Known transfer function zero: z = +1");
        System.out.println("Known poles: -1, -2, -3\n");

        double eta0 = 0.02;
        double[] x = {eta0, eta0, eta0};

        double h = 0.01;
        int steps = (int)(5.0 / h);
        double maxAbsY = 0.0;

        for (int k = 0; k <= steps; k++) {
            double t = k * h;
            double y = output(x);
            maxAbsY = Math.max(maxAbsY, Math.abs(y));

            if (k % 100 == 0) {
                System.out.printf("t=%.6f, eta=x1=%.6f, u=%.6f, y=%.6e%n",
                        t, x[0], outputNullingInput(x), y);
            }

            if (k < steps) {
                x = rk4Step(x, h);
            }
        }

        System.out.printf("%nmax |y(t)| = %.6e%n", maxAbsY);
        System.out.printf("eta(5) numerical = %.9f%n", x[0]);
        System.out.printf("eta(5) exact     = %.9f%n", eta0 * Math.exp(5.0));
    }
}
