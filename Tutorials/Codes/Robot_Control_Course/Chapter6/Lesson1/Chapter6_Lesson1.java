
public class ForceControlMotivation1DOF {

    static double x_d(double t) {
        return (t < 0.1) ? -0.02 : 0.03;
    }

    public static void main(String[] args) {
        double m = 1.0;
        double b = 1.0;
        double k_p = 200.0;
        double k_d = 20.0;
        double k_e = 5000.0;
        double x_s = 0.0;
        double F_max = 80.0;
        double dt = 0.0005;
        double T = 1.0;
        int N = (int) (T / dt);

        double x = -0.02;
        double v = 0.0;

        double[] xs = new double[N];
        double[] Fs = new double[N];

        for (int i = 0; i < N; i++) {
            double t = i * dt;

            double F_e = (x <= x_s) ? 0.0 : k_e * (x - x_s);

            double e = x_d(t) - x;
            double u = k_p * e - k_d * v;

            if (F_e > F_max) {
                u -= (F_e - F_max);
            }

            double a = (u - F_e - b * v) / m;
            v += dt * a;
            x += dt * v;

            xs[i] = x;
            Fs[i] = F_e;
        }

        System.out.println("Final x = " + xs[N - 1]
                + ", Final F_e = " + Fs[N - 1]);
    }
}
