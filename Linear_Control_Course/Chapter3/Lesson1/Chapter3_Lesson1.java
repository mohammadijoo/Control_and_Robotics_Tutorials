public class LTIMassSpring {
    public static void main(String[] args) {
        double m = 1.0;
        double b = 0.4;
        double k = 2.0;

        double[] x = {0.0, 0.0}; // x[0] = q, x[1] = q_dot
        double dt = 0.001;
        int steps = 10000;

        for (int i = 0; i < steps; i++) {
            double u = 1.0; // constant force

            double x1dot = x[1];
            double x2dot = -(k / m) * x[0] - (b / m) * x[1] + (1.0 / m) * u;

            x[0] += dt * x1dot;
            x[1] += dt * x2dot;
        }

        System.out.println("Final position approx: " + x[0]);
    }
}
