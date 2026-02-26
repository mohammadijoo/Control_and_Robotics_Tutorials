public class FirstOrderApprox {
    public static void main(String[] args) {
        double tau = 1.0;
        double K = 1.0;
        double dt = 0.001;
        double T_end = 8.0;
        double x = 0.0;
        double u = 1.0;

        for (double t = 0.0; t <= T_end; t += dt) {
            double dx = -(1.0 / tau) * x + (K / tau) * u;
            x += dt * dx;
            if (((int)(t * 1000)) % 100 == 0) {
                System.out.println(t + " " + x);
            }
        }
    }
}
