public class SinusoidalLtiSimulation {

    public static void main(String[] args) {
        double K = 5.0;
        double tau = 0.1;
        double U = 1.0;
        double omega = 10.0;

        double dt = 1e-4;
        double tFinal = 2.0;
        int steps = (int) (tFinal / dt);

        double y = 0.0;
        double t = 0.0;

        for (int k = 0; k < steps; k++) {
            double u = U * Math.sin(omega * t);
            double dy = (-y + K * u) / tau;

            y += dt * dy;
            t += dt;

            if (k % 1000 == 0) {
                System.out.println(t + " " + u + " " + y);
            }
        }
    }
}
