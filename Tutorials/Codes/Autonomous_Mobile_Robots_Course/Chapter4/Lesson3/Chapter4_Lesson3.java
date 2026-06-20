// Chapter4_Lesson3.java
// Simple dynamic model for a ground robot: velocity-lag unicycle with RK4 integration.
// Pure-Java implementation (no external dependencies). If you prefer a robotics math library,
// you can replace the small vector ops with EJML (org.ejml) or Apache Commons Math.

public class Chapter4_Lesson3 {

    static class Params {
        double T_v = 0.30;
        double T_w = 0.25;
    }

    static double vRef(double t) { return (t >= 0.5) ? 1.0 : 0.0; }
    static double wRef(double t) { return (t >= 2.0) ? 0.7 : 0.0; }

    // x = [px, py, theta, v, w]
    static double[] f(double t, double[] x, Params p) {
        double px = x[0], py = x[1], th = x[2], v = x[3], w = x[4];
        return new double[] {
            v * Math.cos(th),
            v * Math.sin(th),
            w,
            (vRef(t) - v) / p.T_v,
            (wRef(t) - w) / p.T_w
        };
    }

    static double[] add(double[] a, double[] b, double s) {
        double[] out = new double[a.length];
        for (int i = 0; i < a.length; i++) out[i] = a[i] + s * b[i];
        return out;
    }

    static double[] rk4Step(double t, double h, double[] x, Params p) {
        double[] k1 = f(t, x, p);
        double[] k2 = f(t + 0.5*h, add(x, k1, 0.5*h), p);
        double[] k3 = f(t + 0.5*h, add(x, k2, 0.5*h), p);
        double[] k4 = f(t + h, add(x, k3, h), p);
        double[] out = new double[x.length];
        for (int i = 0; i < x.length; i++) {
            out[i] = x[i] + (h/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
        }
        return out;
    }

    public static void main(String[] args) {
        Params p = new Params();
        double[] x = new double[] {0,0,0,0,0};

        double t0 = 0.0, tf = 8.0, h = 0.001;
        double t = t0;
        while (t < tf) {
            x = rk4Step(t, h, x, p);
            t += h;
        }

        System.out.printf("Final pose: px=%.3f py=%.3f theta=%.3f%n", x[0], x[1], x[2]);
        System.out.printf("Final body velocities: v=%.3f w=%.3f%n", x[3], x[4]);
    }
}
