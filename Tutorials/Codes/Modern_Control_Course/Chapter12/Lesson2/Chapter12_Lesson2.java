// Chapter12_Lesson2.java
// Minimum-energy steering for the double-integrator example.
// No external library is required.

public class Chapter12_Lesson2 {
    static double T = 2.0;
    static double pf = 1.0;  // desired final position
    static double vf = 0.0;  // desired final velocity

    // For double integrator from x0 = 0:
    // u*(t) = 6(T - 2t) pf / T^3 + (6t - 2T) vf / T^2
    static double uStar(double t) {
        return 6.0 * (T - 2.0 * t) * pf / Math.pow(T, 3)
             + (6.0 * t - 2.0 * T) * vf / Math.pow(T, 2);
    }

    static double[] f(double t, double[] x) {
        double[] dx = new double[2];
        dx[0] = x[1];
        dx[1] = uStar(t);
        return dx;
    }

    static double[] add(double[] a, double scale, double[] b) {
        return new double[] { a[0] + scale * b[0], a[1] + scale * b[1] };
    }

    static double[] rk4Step(double t, double[] x, double h) {
        double[] k1 = f(t, x);
        double[] k2 = f(t + 0.5 * h, add(x, 0.5 * h, k1));
        double[] k3 = f(t + 0.5 * h, add(x, 0.5 * h, k2));
        double[] k4 = f(t + h, add(x, h, k3));

        return new double[] {
            x[0] + h * (k1[0] + 2.0*k2[0] + 2.0*k3[0] + k4[0]) / 6.0,
            x[1] + h * (k1[1] + 2.0*k2[1] + 2.0*k3[1] + k4[1]) / 6.0
        };
    }

    static double minimumEnergy() {
        // E_min = 12 pf^2/T^3 - 12 pf vf/T^2 + 4 vf^2/T
        return 12.0 * pf * pf / Math.pow(T, 3)
             - 12.0 * pf * vf / Math.pow(T, 2)
             + 4.0 * vf * vf / T;
    }

    public static void main(String[] args) {
        int N = 2000;
        double h = T / N;
        double[] x = {0.0, 0.0};
        double t = 0.0;

        for (int k = 0; k < N; k++) {
            x = rk4Step(t, x, h);
            t += h;
        }

        System.out.printf("minimum energy = %.12f%n", minimumEnergy());
        System.out.printf("terminal state reached = [%.12f, %.12f]%n", x[0], x[1]);
        System.out.printf("target state = [%.12f, %.12f]%n", pf, vf);
        System.out.printf("terminal error norm = %.12e%n",
                Math.sqrt(Math.pow(x[0] - pf, 2) + Math.pow(x[1] - vf, 2)));

        for (int i = 0; i <= 4; i++) {
            double ti = i * T / 4.0;
            System.out.printf("u_star(%.2f) = %.12f%n", ti, uStar(ti));
        }
    }
}
