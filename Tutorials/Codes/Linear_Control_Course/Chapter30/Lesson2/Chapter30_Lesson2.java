public class CoupledPlantSimulation {
    public static void main(String[] args) {
        double alpha = 0.9;
        double k = 2.0;
        double T = 10.0;
        double dt = 1e-3;
        int steps = (int)(T / dt);

        double x1 = 0.0, x2 = 0.0;
        double r1 = 1.0, r2 = 0.0;

        for (int i = 0; i < steps; i++) {
            double t = i * dt;

            double y1 = x1;
            double y2 = x2;

            double e1 = r1 - y1;
            double e2 = r2 - y2;

            double u1 = k * e1;
            double u2 = k * e2;

            double x1dot = -x1 + 1.0 * u1 + alpha * u2;
            double x2dot = -x2 + alpha * u1 + 1.0 * u2;

            x1 += dt * x1dot;
            x2 += dt * x2dot;

            if (i % 100 == 0) {
                System.out.println(t + " " + y1 + " " + y2);
            }
        }
    }
}
