public class ProportionalControllerDemo {

    public static void main(String[] args) {
        double k = 1.0;
        double tau = 0.4;

        double Kp = 10.0;
        double dt = 0.001;
        double tFinal = 3.0;

        double r = 1.0;   // reference
        double y = 0.0;   // output
        double u = 0.0;   // command
        double umax = 12.0;

        int steps = (int) (tFinal / dt);
        for (int i = 0; i < steps; i++) {
            // Proportional control law
            double e = r - y;
            u = Kp * e;

            // Saturation
            if (u > umax)  u = umax;
            if (u < -umax) u = -umax;

            // First-order plant dynamics
            double dy = (-1.0 / tau) * y + (k / tau) * u;
            y += dy * dt;

            // In a real robot, write 'u' to the motor and read 'y' from an encoder
        }

        System.out.println("Final output y = " + y);
    }
}
