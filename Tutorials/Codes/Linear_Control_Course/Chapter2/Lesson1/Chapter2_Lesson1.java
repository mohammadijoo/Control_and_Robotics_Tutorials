public class MassSpringDamperEuler {
    public static void main(String[] args) {
        double m = 1.0, b = 0.2, k = 1.0;
        double dt = 0.001;
        double q = 0.0;
        double qdot = 0.0;

        for (int i = 0; i < 10000; i++) {
            double t = i * dt;
            double u = 1.0;
            double qddot = (u - b * qdot - k * q) / m;

            q    += dt * qdot;
            qdot += dt * qddot;

            if (i % 1000 == 0) {
                System.out.println(t + " " + q);
            }
        }
    }
}

// Java robotics frameworks (e.g., WPILib in mobile robotics) rely on
// numerically integrating ODEs representing drivetrain or arm dynamics.
