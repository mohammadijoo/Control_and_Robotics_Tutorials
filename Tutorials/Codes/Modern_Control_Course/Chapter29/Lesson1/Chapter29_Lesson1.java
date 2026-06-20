// Chapter29_Lesson1.java
// Linear time-varying (LTV) state-space simulation using fixed-step RK4.
// Compile and run:
//     javac Chapter29_Lesson1.java
//     java Chapter29_Lesson1

public class Chapter29_Lesson1 {
    static class Matrices {
        double[][] A = new double[2][2];
        double[] B = new double[2];
        double[] C = new double[2];
        double D;
    }

    static Matrices matrices(double t) {
        double omega = 1.0 + 0.30 * Math.sin(0.70 * t);
        double damping = 0.15 + 0.05 * Math.cos(0.50 * t);

        Matrices m = new Matrices();
        m.A[0][0] = 0.0;
        m.A[0][1] = 1.0;
        m.A[1][0] = -(omega * omega);
        m.A[1][1] = -2.0 * damping * omega;
        m.B[0] = 0.0;
        m.B[1] = 1.0 + 0.20 * Math.sin(0.30 * t);
        m.C[0] = 1.0 + 0.10 * Math.sin(0.40 * t);
        m.C[1] = 0.0;
        m.D = 0.0;
        return m;
    }

    static double inputSignal(double t) {
        return Math.sin(1.2 * t);
    }

    static double[] rhs(double t, double[] x) {
        Matrices m = matrices(t);
        double u = inputSignal(t);
        return new double[] {
            m.A[0][0] * x[0] + m.A[0][1] * x[1] + m.B[0] * u,
            m.A[1][0] * x[0] + m.A[1][1] * x[1] + m.B[1] * u
        };
    }

    static double[] addScaled(double[] x, double[] k, double scale) {
        return new double[] {x[0] + scale * k[0], x[1] + scale * k[1]};
    }

    static double[] rk4Step(double t, double[] x, double h) {
        double[] k1 = rhs(t, x);
        double[] k2 = rhs(t + 0.5 * h, addScaled(x, k1, 0.5 * h));
        double[] k3 = rhs(t + 0.5 * h, addScaled(x, k2, 0.5 * h));
        double[] k4 = rhs(t + h, addScaled(x, k3, h));

        return new double[] {
            x[0] + (h / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
            x[1] + (h / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1])
        };
    }

    public static void main(String[] args) {
        double t0 = 0.0;
        double tf = 20.0;
        double h = 0.01;
        double[] x = {1.0, 0.0};

        System.out.println("t,x1,x2,y");
        for (double t = t0; t <= tf + 1e-12; t += h) {
            Matrices m = matrices(t);
            double u = inputSignal(t);
            double y = m.C[0] * x[0] + m.C[1] * x[1] + m.D * u;

            int step = (int) Math.round(t / h);
            if (step % 100 == 0) {
                System.out.printf("%.8f,%.8f,%.8f,%.8f%n", t, x[0], x[1], y);
            }

            if (t + h <= tf + 1e-12) {
                x = rk4Step(t, x, h);
            }
        }
    }
}
