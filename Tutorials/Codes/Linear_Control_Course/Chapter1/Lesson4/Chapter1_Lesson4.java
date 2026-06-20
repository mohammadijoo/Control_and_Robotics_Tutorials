public class FirstOrderControlSim {

    // In robotics, similar logic could be integrated into a Java-based
    // middleware (e.g. rosjava) to control actuators.

    public static void main(String[] args) {
        double a = 1.0;
        double b = 1.0;
        double k = 5.0;
        double r = 1.0;

        double dt = 0.001;
        double tFinal = 5.0;
        int nSteps = (int) (tFinal / dt);

        double x = 0.0;
        double x_ss = 0.0;
        double maxU = 0.0;

        for (int i = 0; i < nSteps; i++) {
            double e = r - x;
            double u = k * e;
            double dx = -(a + b * k) * x + b * k * r;
            x += dt * dx;

            if (Math.abs(u) > maxU) {
                maxU = Math.abs(u);
            }

            x_ss = x;
        }

        double e_ss = r - x_ss;
        System.out.println("Approx steady-state output y_ss = " + x_ss);
        System.out.println("Approx steady-state error e_ss = " + e_ss);
        System.out.println("Peak control effort max|u|    = " + maxU);
    }
}
