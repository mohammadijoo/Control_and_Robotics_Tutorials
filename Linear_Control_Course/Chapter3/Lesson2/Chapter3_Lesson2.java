public class MassSpringDamperSim {
    public static void main(String[] args) {
        double m = 1.0;
        double b = 0.4;
        double k = 4.0;

        double y = 0.0;  // displacement
        double v = 0.0;  // velocity

        double dt = 0.001;
        double tFinal = 10.0;
        int steps = (int) (tFinal / dt);

        for (int i = 0; i < steps; i++) {
            double t = i * dt;

            // Unit step force
            double F = 1.0;

            // Acceleration from m * y_ddot + b * y_dot + k * y = F
            double a = (F - b * v - k * y) / m;

            // Euler integration
            v = v + dt * a;
            y = y + dt * v;

            if (i % 1000 == 0) {
                System.out.printf("t = %.3f, y = %.5f%n", t, y);
            }
        }
    }
}
